// SPDX-License-Identifier: MIT OR Apache-2.0
// Batched descriptor top-2 nearest-neighbour search for visloc-rs matching.
//
// This is an independent CUDA implementation of the exact nearest-neighbour +
// second-best contract the Rust `BruteForceMatcher` uses: for every query row,
// the best and second-best L2 distances over the train rows, keyed by train
// index. It does not depend on cuBLAS or PyTorch.
//
// The work is O(N_query x N_train x dim), the same as the CPU GEMM, but the
// distance matrix never leaves the device: only the per-query top-2 results are
// copied back (N_query x 4 scalars).

#include <cuda_runtime.h>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <limits>

#ifdef _WIN32
#define VISLOC_EXPORT extern "C" __declspec(dllexport)
#else
#define VISLOC_EXPORT extern "C" __attribute__((visibility("default")))
#endif

namespace {

constexpr uint32_t kAbiVersion = 1;

struct Context {
  float* query = nullptr;
  float* train = nullptr;
  // Per-query output: best_distance, best_index(as float), second_distance,
  // second_index(as float). Four floats per query.
  float* output = nullptr;
  size_t query_capacity = 0;
  size_t train_capacity = 0;
  size_t output_capacity = 0;
  char error[512] = {};
};

bool reserve(void** pointer, size_t* capacity, size_t bytes, Context* context,
             const char* label) {
  if (*capacity >= bytes) return true;
  if (*pointer) cudaFree(*pointer);
  const cudaError_t status = cudaMalloc(pointer, bytes);
  if (status != cudaSuccess) {
    std::snprintf(context->error, sizeof(context->error),
                  "cudaMalloc(%s, %zu): %s", label, bytes,
                  cudaGetErrorString(status));
    *pointer = nullptr;
    *capacity = 0;
    return false;
  }
  *capacity = bytes;
  return true;
}

// One thread per query row. Each thread walks every train row and keeps the
// best two squared distances (ties broken by the lower train index, matching the
// strict-`<` first-wins CPU tie-break). N_train is expected to be a few thousand,
// so a serial walk per query is memory-bandwidth-bound and simple to verify.
__global__ void top2_kernel(const float* __restrict__ query,
                            const float* __restrict__ train, int n_query,
                            int n_train, int dim, float* __restrict__ output) {
  const int query_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (query_index >= n_query) return;

  const float* q = query + static_cast<size_t>(query_index) * dim;
  float best = 0.0f;
  float second = 0.0f;
  int best_index = -1;
  int second_index = -1;

  for (int train_index = 0; train_index < n_train; ++train_index) {
    const float* t = train + static_cast<size_t>(train_index) * dim;
    float sum = 0.0f;
    for (int k = 0; k < dim; ++k) {
      const float diff = q[k] - t[k];
      sum = fmaf(diff, diff, sum);
    }
    if (best_index < 0 || sum < best) {
      second = best;
      second_index = best_index;
      best = sum;
      best_index = train_index;
    } else if (second_index < 0 || sum < second) {
      second = sum;
      second_index = train_index;
    }
  }

  float* out = output + static_cast<size_t>(query_index) * 4;
  out[0] = best_index < 0 ? 0.0f : sqrtf(fmaxf(best, 0.0f));
  out[1] = static_cast<float>(best_index);
  out[2] = second_index < 0 ? -1.0f : sqrtf(fmaxf(second, 0.0f));
  out[3] = static_cast<float>(second_index);
}

}  // namespace

extern "C" {

VISLOC_EXPORT uint32_t visloc_descriptor_gemm_abi_version() {
  return kAbiVersion;
}

VISLOC_EXPORT void* visloc_descriptor_gemm_create() {
  return new (std::nothrow) Context();
}

VISLOC_EXPORT void visloc_descriptor_gemm_destroy(void* raw) {
  if (!raw) return;
  Context* context = static_cast<Context*>(raw);
  if (context->query) cudaFree(context->query);
  if (context->train) cudaFree(context->train);
  if (context->output) cudaFree(context->output);
  delete context;
}

VISLOC_EXPORT const char* visloc_descriptor_gemm_last_error(void* raw) {
  if (!raw) return "null context";
  return static_cast<Context*>(raw)->error;
}

// Runs the search. `query` and `train` are row-major host buffers
// (n_query x dim) and (n_train x dim). `output` receives 4 floats per query.
// Returns 0 on success, non-zero on error (see last_error).
VISLOC_EXPORT int visloc_descriptor_gemm_run(void* raw, const float* query,
                                             int n_query, const float* train,
                                             int n_train, int dim,
                                             float* output) {
  Context* context = static_cast<Context*>(raw);
  if (!context) return 1;
  context->error[0] = '\0';
  if (n_query <= 0 || n_train <= 0 || dim <= 0) {
    std::snprintf(context->error, sizeof(context->error),
                  "invalid shape n_query=%d n_train=%d dim=%d", n_query, n_train,
                  dim);
    return 2;
  }
  if (!query || !train || !output) {
    std::snprintf(context->error, sizeof(context->error), "null buffer");
    return 3;
  }

  const size_t query_bytes = static_cast<size_t>(n_query) * dim * sizeof(float);
  const size_t train_bytes = static_cast<size_t>(n_train) * dim * sizeof(float);
  const size_t output_bytes = static_cast<size_t>(n_query) * 4 * sizeof(float);
  if (!reserve(reinterpret_cast<void**>(&context->query), &context->query_capacity,
               query_bytes, context, "query") ||
      !reserve(reinterpret_cast<void**>(&context->train), &context->train_capacity,
               train_bytes, context, "train") ||
      !reserve(reinterpret_cast<void**>(&context->output),
               &context->output_capacity, output_bytes, context, "output")) {
    return 4;
  }

  cudaError_t status = cudaMemcpy(context->query, query, query_bytes,
                                  cudaMemcpyHostToDevice);
  if (status != cudaSuccess) {
    std::snprintf(context->error, sizeof(context->error),
                  "cudaMemcpy H2D(query): %s", cudaGetErrorString(status));
    return 5;
  }
  status = cudaMemcpy(context->train, train, train_bytes,
                      cudaMemcpyHostToDevice);
  if (status != cudaSuccess) {
    std::snprintf(context->error, sizeof(context->error),
                  "cudaMemcpy H2D(train): %s", cudaGetErrorString(status));
    return 6;
  }

  const int threads = 128;
  const int blocks = (n_query + threads - 1) / threads;
  top2_kernel<<<blocks, threads>>>(context->query, context->train, n_query,
                                   n_train, dim, context->output);
  status = cudaGetLastError();
  if (status != cudaSuccess) {
    std::snprintf(context->error, sizeof(context->error),
                  "top2 kernel launch: %s", cudaGetErrorString(status));
    return 7;
  }
  status = cudaMemcpy(output, context->output, output_bytes,
                      cudaMemcpyDeviceToHost);
  if (status != cudaSuccess) {
    std::snprintf(context->error, sizeof(context->error),
                  "cudaMemcpy D2H(output): %s", cudaGetErrorString(status));
    return 8;
  }
  return 0;
}

}  // extern "C"
