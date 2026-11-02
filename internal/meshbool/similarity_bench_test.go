package meshbool

import (
	"testing"

	"github.com/lestrrat-3d/r3"
)

var exactSimilarityImageResult any

func BenchmarkExactSimilarityImage(b *testing.B) {
	anchor := r3.NewVec(1, 2, 3)
	point := r3.NewVec(4.1, 5.2, 6.3)
	for b.Loop() {
		exactSimilarityImageResult = ExactSimilarityImage(anchor, point, 2)
	}
}
