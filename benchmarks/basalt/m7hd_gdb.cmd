set pagination off
set confirm off
break "Eigen::internal::generic_product_impl<Eigen::Product<Eigen::Matrix<float, 9, 3, 0, 9, 3>, Eigen::DiagonalWrapper<Eigen::Matrix<float, 3, 1, 0, 3, 1> const>, 1>, Eigen::Transpose<Eigen::Matrix<float, 9, 3, 0, 9, 3> const>, Eigen::DenseShape, Eigen::DenseShape, 8>::scaleAndAddTo<Eigen::Matrix<float, 9, 9, 0, 9, 9> >"
commands
 silent
 printf "kernel rdi=%p rsi=%p rdx=%p rcx=%p\\n", $rdi,$rsi,$rdx,$rcx
 x/12wx $rsi
 x/12wx $rdx
 continue
end
run target/m7hd_native_intermediate.jsonl
