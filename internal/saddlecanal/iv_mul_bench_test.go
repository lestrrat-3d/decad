package saddlecanal

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/r3"
)

func BenchmarkProveSaddleFillet(b *testing.B) {
	c := Canal{R: 10, Bore: 3, Rho: 1.5}
	rows, err := c.Rows()
	if err != nil {
		b.Fatal(err)
	}
	pose := crossbore.Pose{Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1}}
	held := func(k, j int) r3.Vec { return c.Point(k, rows[j]) }
	b.ResetTimer()
	for range b.N {
		if _, err := Prove(b.Context(), c, rows, pose, held); err != nil {
			b.Fatal(err)
		}
	}
}
