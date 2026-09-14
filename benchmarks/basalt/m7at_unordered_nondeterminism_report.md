# M7at unordered landmark traversal audit

The pinned Basalt binary was replayed twice on MH01 through frame 4 with a
fresh `BASALT_VIO_ITERATION_TRACE` path for each run.  Both traces contained
61 landmarks for frame 4.  The `landmark_linearization` and `landmark_qr`
records are emitted from TBB workers, so their JSONL line order is not a
stable solver contract:

* run A linearization emission began `14, 3, 18, 49, 119, 108, 11, 117, ...`
* run B linearization emission began `75, 84, 100, 91, 66, 18, 109, 93, ...`

The two runs have the same landmark set and the same `landmark_qr.row_span`
mapping.  Sorting QR records by the assigned row start yields the same
container sequence in both runs:

```text
120 119 21 14 73 13 72 3 131 4 63 10 128 1 30 11 40 99 12 37 29 58 117 55 54 20 49 108 19 48 18 47 46 9 38 2 31 90 22 23 82 27 28 87 64 66 74 75 83 84 91 92 93 100 101 102 103 109 110 111 118
```

This is evidence that a fixed sidecar for unordered traversal is not a
faithful production contract.  Production assembly therefore keeps the
canonical Rust landmark representation; trace comparison treats worker
emission order as unordered and compares factor payloads by track id.  No
threshold, acceptance decision, or golden metric was changed.
