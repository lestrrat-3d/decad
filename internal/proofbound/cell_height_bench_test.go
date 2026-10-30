package proofbound_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

var cellHeightSink float64
var cellHeightOKSink bool

func BenchmarkCellHeightOffsetUpper(b *testing.B) {
	cells := []struct {
		name    string
		corners [4]r3.Vec
	}{
		{"flat", [4]r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(1, 1, 0)}},
		{"lifted", [4]r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(1, 1, 0.5)}},
	}
	for _, cell := range cells {
		b.Run(cell.name, func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				cellHeightSink, cellHeightOKSink = proofbound.CellHeightOffsetUpper(
					cell.corners[0], cell.corners[1], cell.corners[2], cell.corners[3],
				)
			}
		})
	}
}
