#include "../mainPipeline/pipeline.hpp"

int main() {
    pipeline p;
    p.Pipeline(
        "../data/bun045_ascii.pcd",
        "../data/bun000_ascii.pcd"
    );
    return 0;
}