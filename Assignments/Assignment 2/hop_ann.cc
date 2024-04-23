#include<stdio.h>
#include<string.h>
#include<algorithm>
#include<queue>
#include <limits>
#include <omp.h>

#define max_threads 8

int V,D,E,L,K,A,B,C,M,Q;
int* X;
int* edges;

// Define a struct to hold query information
struct QueryInfo {
    int start_point;
    int hop;
    int* query_data;
    int nearest_id;
};

// Define the visited to track the already visited vertices
int* visited[max_threads];
std::vector<int> visited_flag(max_threads, 0);

int squared_l2_dist(int* x,int* y,int D){
	int sum2 = 0;
	for(int i = 0;i < D;++i)
		sum2 += (x[i] - y[i]) * (x[i] - y[i]);
	return sum2;
}

int nearest_id_small(int start_point, int max_hop, int* query_data){
	int min_d = std::numeric_limits<int>::max();
	int min_id = -1;
    std::queue<std::pair<int, int>> q;
	q.push(std::make_pair(start_point, 0));
	while(!q.empty()){
		auto now = q.front();
		q.pop();
		int id = now.first;
		int hop = now.second;
		int d = squared_l2_dist(X + id * D, query_data, D);
		if((d < min_d) || (d == min_d && id < min_id)){
			min_d = d;
			min_id = id;
		}
		if(hop + 1 <= max_hop){
			int degree = edges[id * (L + 1)];
			for(int i = 1;i <= degree;++i){
				int v = edges[id * (L + 1) + i];
				q.push(std::make_pair(v, hop + 1));
			}
		}
	}
	return min_id;
}
// keep track off visisted nodes with large queries to avoid revisits
int nearest_id_large(int start_point, int max_hop, int* query_data, int qid, int tid) {
    int min_d = std::numeric_limits<int>::max();
    int min_id = -1;
    std::queue<std::pair<int, int>> q;
    q.push(std::make_pair(start_point, max_hop));
    visited[tid][start_point] = ++visited_flag[tid];
    while (!q.empty()) {
        auto now = q.front();
        q.pop();
        int id = now.first;
        int hop = now.second;
        int d = squared_l2_dist(X + id * D, query_data, D);
        if ((d < min_d) || (d == min_d && id < min_id)) {
            min_d = d;
            min_id = id;
        }
        if (hop - 1 >= 0) {
            int degree = edges[id * (L + 1)];
            for (int i = 1; i <= degree; ++i) {
                int v = edges[id * (L + 1) + i];
                if (visited[tid][v] != visited_flag[tid]) {
                    visited[tid][v] = visited_flag[tid];
                    q.push(std::make_pair(v, hop - 1));
                }
            }
        }
    }
    return min_id;
}

int main(int argc,char** argv){
	FILE* fin = fopen(argv[1],"r");
	FILE* fout = fopen(argv[2],"w");
	fscanf(fin,"%d%d%d%d%d%d%d%d%d%d",&V,&D,&E,&L,&K,&A,&B,&C,&M,&Q);
	X = new int[V * D];
	for(int i = 0;i < K;++i)
		fscanf(fin,"%d",&X[i]);
	for(int i = K;i < V * D;++i)
		X[i] = ((long long)A * X[i - 1] + (long long)B * X[i - 2] + C) % M;
	edges = new int[V * (L + 1)];
	for(int i = 0;i < V;++i){
		edges[i * (L + 1)] = 0;
	}
	for(int i = 0;i < E;++i){
		int u,v;
		fscanf(fin,"%d%d",&u,&v);
		int degree = edges[u * (L + 1)];
		edges[u * (L + 1) + degree + 1] = v;
		++edges[u * (L + 1)];
	}
    // Read All queries first into memory for parallel execution on data
    std::vector<QueryInfo> queries(Q);
    for (int i = 0; i < Q; ++i) {
        fscanf(fin, "%d%d", &queries[i].start_point, &queries[i].hop);
        queries[i].query_data = new int[D];
        for (int j = 0; j < D; ++j) {
            fscanf(fin, "%d", &queries[i].query_data[j]);
        }
    }
    // use the basic (without multi-thread and visited track) for small query dimensions to avoid overhead
    if (D <= 10){
        for (int i = 0; i < Q; ++i)
            queries[i].nearest_id = nearest_id_small(queries[i].start_point, queries[i].hop, queries[i].query_data);
    }
    else {
        // Allocate memory for visited array of size V for each thread to 0
        for (int i = 0; i < max_threads; ++i) {
            visited[i] = new int[V];
            memset(visited[i], 0, sizeof(int) * V);
        }
        #pragma omp parallel for num_threads(max_threads)
        for (int i = 0; i < Q; ++i) {
            int tid = omp_get_thread_num();
            queries[i].nearest_id = nearest_id_large(queries[i].start_point, queries[i].hop, queries[i].query_data, i, tid);
        }
    }
    
    // Write results to the output file in order
    for (int i = 0; i < Q; ++i) {
        fprintf(fout, "%d\n", queries[i].nearest_id);
        // Don't forget to free memory
        delete[] queries[i].query_data; 
    }

	fclose(fin);
	fclose(fout);

	delete[] X;
	delete[] edges;

	return 0;
}