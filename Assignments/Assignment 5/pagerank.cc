#include<stdio.h>
#include<string.h>
#include<algorithm>
#include<queue>
#include<vector>
#include<omp.h>

const double d = 0.85;
int V,E,L,M;

std::vector<std::vector<int>> in_edges;
std::vector<int> out_degree;

int main(int argc,char** argv){
	FILE* fin = fopen(argv[1],"r");
	FILE* fout = fopen(argv[2],"w");
	fscanf(fin,"%d%d%d%d",&V,&E,&L,&M);
	in_edges.resize(V);
	out_degree = std::vector<int>(V,0);

	for(int i = 0;i < E;++i){
		int u,v;
		fscanf(fin,"%d%d",&u,&v);
		in_edges[v].push_back(u);
		++out_degree[u];
	}

	std::vector<double> pr[2];
	pr[0].resize(V);
	pr[1].resize(V);
	int current = 0;
	for(int i = 0;i < V;++i){
		pr[current][i] = 1.0 / V;
	}
	for(int iter = 0;iter < M;++iter){
		int next = 1 - current;
        #pragma omp parallel for
		for(int i = 0;i < V;++i){
			double sum = 0;
			for(int j = 0;j < in_edges[i].size();++j){
				int v = in_edges[i][j];
				sum += pr[current][v] / out_degree[v];
			}
			pr[next][i] = (1.0 - d) / V + d * sum;
		}
		current = next;
	}

	for(int i = 0;i < V;++i){
		fprintf(fout,"%.8f\n",pr[current][i]);
	}
	fclose(fin);
	fclose(fout);

	return 0;
}
