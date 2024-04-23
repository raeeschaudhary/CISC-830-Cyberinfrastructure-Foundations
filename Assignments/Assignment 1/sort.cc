#include<stdio.h>
#include<omp.h>
#include<algorithm>
using namespace std;
#define MAX_THREADS 24

// Sequential merge function
void merge(int X[], int l, int m, int r) {
    // create L(left) and R(right) array with size of n1, n2
    int n1 = m - l + 1, n2 = r - m;
    int* L = new int[n1];
    int* R = new int[n2];
    // sort L and R
    for (int i = 0; i < n1; i++)
        L[i] = X[l + i];
    for (int j = 0; j < n2; j++)
        R[j] = X[m + 1 + j];

    int i = 0, j = 0, k = l;
    // do the merging 
    while (i < n1 && j < n2) {
        if (L[i] <= R[j]) {
            X[k++] = L[i++];
        } else {
            X[k++] = R[j++];
        }
    }
    // do the if something is remaining
    while (i < n1) {
        X[k++] = L[i++];
    }
    while (j < n2) {
        X[k++] = R[j++];
    }
}

// perform sort and merge
void mergeSort(int X[], int l, int r){
	if (l < r) {
		int m = l + (r - l) / 2;
		mergeSort(X, l, m);
		mergeSort(X, m + 1, r);
		merge(X, l, m, r);
	}
}

// Parallel merge sort function
void mergeSortParallel(int X[], int l, int r) {
        if (l < r) {
        int m = l + (r - l) / 2;
        // parallel sections with max threads
        int num_threads = min(MAX_THREADS, omp_get_max_threads());
        #pragma omp parallel sections num_threads(num_threads) 
            {
				#pragma omp section
					{
						mergeSort(X, l, m);
					}
				#pragma omp section
					{
						mergeSort(X, m + 1, r);
					}
            }
        merge(X, l, m, r);
    }
}

int X[100000000];

int main(int argc,char** argv){
	int N,K,A,B,C,M;
	FILE* fin = fopen(argv[1],"r");
	fscanf(fin,"%d%d%d%d%d%d",&N,&K,&A,&B,&C,&M);
	for(int i = 0;i < K;++i)
		fscanf(fin,"%d",&X[i]);
	fclose(fin);

	FILE* fout = fopen(argv[2],"w");
	for(int i = K;i < N;++i)
		X[i] = ((long long)A * X[i - 1] + (long long)B * X[i - 2] + C) % M;
	// std::sort(X,X + N);
	mergeSortParallel(X, 0, N - 1);
	for(int i = 0;i < N;++i)
		fprintf(fout,"%d\n",X[i]);
	fclose(fout);
	return 0;
}
