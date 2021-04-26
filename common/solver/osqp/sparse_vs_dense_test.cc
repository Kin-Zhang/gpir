// g++ -I/usr/include/eigen3 sparse_vs_dense_test.cc -o test -O3

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <chrono>
#include <iostream>

using namespace std;
using namespace std::chrono;
using namespace Eigen;

int main(int argc, char *argv[]) {
  int size = 1000;
  if (argv[1]) size = atoi(argv[1]);
  printf("problem size: %d x %d\n", size, size);

  auto t1 = high_resolution_clock::now();
  SparseMatrix<double, ColMajor> sparse(size, size);
  for (int i = 0; i < size - 1; ++i) {
    sparse.insert(i, i) = 1;
    sparse.insert(i, i + 1) = 1;
    sparse.insert(i + 1, i) = 1;
  }
  auto t2 = high_resolution_clock::now();
  std::cout << "sparse: " << duration_cast<microseconds>(t2 - t1).count()
            << endl;

  t1 = high_resolution_clock::now();
  MatrixXd dense1 = MatrixXd::Zero(size, size);
  for (int i = 0; i < size - 1; ++i) {
    dense1(i, i) = 1;
    dense1(i, i + 1) = 1;
    dense1(i + 1, i) = 1;
  }
  auto a1 = dense1.sparseView();
  t2 = high_resolution_clock::now();
  std::cout << "dense1: " << duration_cast<microseconds>(t2 - t1).count()
            << endl;

  t1 = high_resolution_clock::now();
  MatrixXd dense2(size, size);
  for (int i = 0; i < size - 1; ++i) {
    dense2(i, i) = 1;
    dense2(i, i + 1) = 1;
    dense2(i + 1, i) = 1;
  }
  auto a2 = dense2.sparseView();
  t2 = high_resolution_clock::now();
  std::cout << "dense2: " << duration_cast<microseconds>(t2 - t1).count()
            << endl;
  return 0;
}
