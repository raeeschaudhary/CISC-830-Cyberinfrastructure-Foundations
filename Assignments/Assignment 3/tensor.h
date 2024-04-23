#include<vector>
#include<stdexcept>
#include <omp.h>

class Tensor{
public:
  std::vector<double> data;
  std::vector<size_t> dims;
  
  Tensor(std::vector<size_t> dims) : dims(dims){
    size_t len = 1;
    for(auto d : dims)
      len *= d;
    data.resize(len);
  }

  Tensor(std::vector<size_t> dims,std::vector<std::vector<size_t>> idx,std::vector<double> val) : dims(dims){
    size_t len = 1;
    for(auto d : dims)
      len *= d;
    data.resize(len);
    if(idx.size() != val.size())
      throw std::runtime_error("Mismatched idx and val size");
    for(size_t i = 0;i < idx.size();++i){
      data[index(idx[i])] = val[i];
    }
  }

  static Tensor ones(std::vector<size_t> dims){
    Tensor ret(dims);
    for(size_t i = 0;i < ret.data.size();++i)
      ret.data[i] = 1;
    return ret;
  }

  size_t index(std::vector<size_t> x){
    if(x.size() != dims.size())
      throw std::runtime_error("Mismatched dims in index");
    size_t ret = 0;
    size_t prod = 1;
    for(int i = dims.size() - 1;i >= 0;--i){
      if(x[i] >= dims[i])
        throw std::runtime_error("Index out of bound");
      ret += x[i] * prod;
      prod *= dims[i];
    } 
    return ret;
  }

  Tensor reshape(std::vector<size_t> new_dims){
    size_t len = 1;
    for(auto d : new_dims)
      len *= d;
    if(len != data.size())
      throw std::runtime_error("Mismatched dims in reshape");
    Tensor ret(new_dims);
    ret.data = data;
    return ret;
  }

  Tensor transpose(){
    if(dims.size() == 2){
      Tensor ret({dims[1],dims[0]});
      for(size_t i = 0;i < dims[0];++i){
        for(size_t j = 0;j < dims[1];++j){
          ret.data[ret.index({j,i})] = data[index({i,j})];
        }
      }
      return ret;
    }else if(dims.size() == 3){
      Tensor ret({dims[0],dims[2],dims[1]});
      for(size_t b = 0;b < dims[0];++b){
        for(size_t i = 0;i < dims[1];++i){
          for(size_t j = 0;j < dims[2];++j){
            ret.data[ret.index({b,j,i})] = data[index({b,i,j})];
          }
        }
      }
      return ret;
    }else{
      throw std::runtime_error("The tensor must be 2D or batched 2D tensors");
    }
  }

  Tensor neg(){
    Tensor ret(dims);
    for(size_t i = 0;i < data.size();++i)
      ret.data[i] = -data[i];
    return ret;
  }
  
  Tensor reciprocal(){
    Tensor ret(dims);
    for(size_t i = 0;i < data.size();++i)
      ret.data[i] = 1.0 / data[i];
    return ret;
  }

  Tensor add(Tensor x){
    if(dims != x.dims)
      throw std::runtime_error("Mismatched shape in add");
    Tensor ret(dims);
    for(size_t i = 0;i < data.size();++i)
      ret.data[i] = data[i] + x.data[i];
    return ret;
  }
  
  Tensor subtract(Tensor x){
    if(dims != x.dims)
      throw std::runtime_error("Mismatched shape in subtract");
    return add(x.neg());
  }

  Tensor mult(double x){
    Tensor ret(dims);
    for(size_t i = 0;i < data.size();++i)
      ret.data[i] = data[i] * x;
    return ret;
  }
  
  Tensor elementwise_mult(Tensor x){
    if(dims != x.dims)
      throw std::runtime_error("Mismatched shape in elementwise_mult");
    Tensor ret(dims);
    for(size_t i = 0;i < data.size();++i)
      ret.data[i] = data[i] * x.data[i];
    return ret;
  }
  
  Tensor pow(double x){
    Tensor ret(dims);
    for(size_t i = 0;i < data.size();++i)
      ret.data[i] = std::pow(data[i],x);
    return ret;
  }
  
  Tensor relu(){
    Tensor ret(dims);
    for(size_t i = 0;i < data.size();++i)
      ret.data[i] = data[i] > 0 ? data[i] : 0;
    return ret;
  }

  Tensor binarilize(){
    Tensor ret(dims);
    for(size_t i = 0;i < data.size();++i)
      ret.data[i] = data[i] > 0 ? 1 : 0;
    return ret;
  }

  Tensor exp(){
    Tensor ret(dims);
    for(size_t i = 0;i < data.size();++i)
      ret.data[i] = std::exp(data[i]);
    return ret;

  }

  // Converts to a sparse representation to optimize multiplication.
  std::vector<std::vector<std::pair<size_t, double>>> mat_csr() {
      std::vector<std::vector<std::pair<size_t, double>>> csr_rep(dims[0]);
      
      #pragma omp parallel for
      for (size_t i = 0; i < dims[0]; ++i) {
          for (size_t j = 0; j < dims[1]; ++j) {
              if (data[index({i, j})] != 0) { // Filter out zero values for sparse representation
                  // Create a pair object representing the non-zero element
                  std::pair<size_t, double> element(j, data[index({i, j})]);
                  // Add the element to the end of the row vector
                  csr_rep[i].push_back(element);
              }
          }
          // Sort the elements of each row based on the column indices
          // std::sort(csr_rep[i].begin(), csr_rep[i].end());
      }
      return csr_rep;
  }

  std::vector<std::vector<std::vector<std::pair<size_t, double>>>> mat_csr_batched() {
    std::vector<std::vector<std::vector<std::pair<size_t, double>>>> csr_batched_rep(dims[0]);

    // Iterate over each batch
    for (size_t b = 0; b < dims[0]; ++b) {
        csr_batched_rep[b].resize(dims[1]);
        
        #pragma omp parallel for
        for (size_t i = 0; i < dims[1]; ++i) {
            for (size_t j = 0; j < dims[2]; ++j) {
                double value = data[index({ b, i, j })]; // Access value in batched matrix
                if (value != 0) { // Filter out zero values for sparse representation
                    // Add the element to the end of the batched row vector
                    csr_batched_rep[b][i].push_back({ j, value });
                }
            }
            // Sort the elements of each row based on the column indices
            // std::sort(csr_batched_rep[b][i].begin(), csr_batched_rep[b][i].end());
        }
    }
    return csr_batched_rep;
}

  Tensor matmul(Tensor x) {
    if(x.dims.size() != 2){
      throw std::runtime_error("The right operand of matmul must be 2D tensors");
    }
    if(dims.size() != 2 && dims.size() != 3){
      throw std::runtime_error("The left operand of matmul must be 2D tensors or batched 2D tensors");
    }
    if(dims[dims.size() - 1] != x.dims[0]){
      throw std::runtime_error("Mismatched matmul matrix dimentions");
    }

    if(dims.size() == 2){
      // Result dimensions: (rows) x (columns)
      std::vector<std::vector<double>> result(dims[0], std::vector<double>(x.dims[1], 0.0));

      // Convert the matrices to a list of pairs (column index, value) for non-zero elements
      auto csr_mat1 = mat_csr();
      auto csr_mat2 = x.mat_csr();

      // Perform multiplication using the sparse representation and parallelize with open mp
      #pragma omp parallel for
      for (size_t i = 0; i < dims[0]; ++i) {
          for (auto& pair1 : csr_mat1[i]) {
              size_t k = pair1.first;
              double value1 = pair1.second;
              for (auto& pair2 : csr_mat2[k]) {
                  size_t j = pair2.first;
                  double value2 = pair2.second;
                  // #pragma omp atomic
                  result[i][j] += value1 * value2;
              }
          }
      }
      // Convert the result back to a Tensor object
      Tensor ret({dims[0], x.dims[1]});
      for (size_t i = 0; i < dims[0]; ++i) {
          for (size_t j = 0; j < x.dims[1]; ++j) {
              ret.data[ret.index({i, j})] = result[i][j];
          }
      }
      return ret;
  }else{
          Tensor ret({ dims[0], dims[1], x.dims[1] });

          // Convert the left and right operand to CSR representation
          auto csr_this = mat_csr_batched();
          auto csr_x = x.mat_csr();

          #pragma omp parallel for
          for (size_t b = 0; b < dims[0]; ++b) {
              for (size_t i = 0; i < dims[1]; ++i) {
                  for (size_t j = 0; j < x.dims[1]; ++j) {
                      double sum = 0.0;
                      for (size_t l = 0; l < dims[2]; ++l) {
                          // Access value in CSR representation of left
                          for (const auto& entry_this : csr_this[b][i]) {
                              if (entry_this.first == l) {
                                  // Access value in CSR representation of right
                                  for (const auto& entry_x : csr_x[l]) {
                                      if (entry_x.first == j) {
                                          sum += entry_this.second * entry_x.second;
                                          break;
                                      }
                                  }
                                  break;
                              }
                          }
                      }
                      ret.data[ret.index({ b, i, j })] = sum;
                  }
              }
          }
          return ret;
        }
  }

  void print(){
    for(auto x : data)
      printf("%s\n",std::to_string(x).c_str());
  }

  std::vector<double> get_data(){
    return data;
  }

  std::vector<size_t> get_dims(){
    return dims;
  }
  
};
