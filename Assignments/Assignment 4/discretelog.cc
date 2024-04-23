#include<stdio.h>
#include<string.h>
#include<algorithm>
#include<queue>
#include<vector>
#include <unordered_map>
#include <cmath>

typedef __int128 int128_t;

long long baby_step_giant_step(long long a, long long b, long long m) {
    a %= m, b %= m;
    long long n = sqrt(m) + 1;

    int128_t an = 1;
    for (long long i = 0; i < n; ++i) {
        an = (an * a) % m;
    }

    std::unordered_map<long long, long long> vals;
    for (long long q = 0, cur = b; q <= n; ++q) {
        vals[cur] = q;
        cur = (cur * a) % m;
    }

    int128_t cur = 1;
    for (long long p = 1; p <= n; ++p) {
        cur = (cur * an) % m;
        if (vals.count(cur)) {
            long long ans = n * p - vals[cur];
            return ans;
        }
    }
}

int main(int argc, char** argv) {
    long long A, B, M;
    FILE* fin = fopen(argv[1], "r");
    FILE* fout = fopen(argv[2], "w");
    fscanf(fin, "%lld%lld%lld", &A, &B, &M);

    long long result = baby_step_giant_step(A, B, M);

    fprintf(fout, "%lld\n", result);

    fclose(fin);
    fclose(fout);

    return 0;
}
