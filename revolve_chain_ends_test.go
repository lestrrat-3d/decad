package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRevolveChainBuiltEnds(t *testing.T) {
	for _, collinear := range []bool{false, true} {
		t.Run(map[bool]string{false: "junction", true: "coalesced"}[collinear], func(t *testing.T) {
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			a, b := s.CreatePoint(3, 0), s.CreatePoint(3, 20)
			x := 5.0
			if collinear {
				x = 3
			}
			c := s.CreatePoint(x, 40)
			s.Fix(a)
			s.CreateLine(a, b)
			s.CreateLine(b, c)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			chains := s.Chains()
			require.Len(t, chains, 1)
			require.Len(t, chains[0].Edges, 2)
			body, err := Revolve(t.Context(), New(), s, chains[0], SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 0, V: 1}}, AngleExtent{A: units.Degrees(90), Dir: Along})
			require.NoError(t, err)
			if collinear {
				require.Len(t, body.Faces(), 1)
			} else {
				require.Len(t, body.Faces(), 2)
			}
			var ends []*Edge
			var foundStart, foundEnd bool
			for _, edge := range body.Edges() {
				ci, si, start, err := revolveplan.FreeEnd(t.Context(), proofbound.NewWorkBudget(t.Context()), &body.n, &edge.n)
				if err != nil {
					continue
				}
				require.Equal(t, 0, ci)
				if start {
					require.Equal(t, 0, si)
					foundStart = true
				} else {
					require.Equal(t, 1, si)
					foundEnd = true
				}
				ends = append(ends, edge)
			}
			require.True(t, foundStart)
			require.True(t, foundEnd)
			require.Len(t, ends, 2)
			if !collinear {
				internal := 0
				for _, edge := range body.Edges() {
					if len(edge.Faces()) != 2 {
						continue
					}
					internal++
					_, _, _, err := revolveplan.FreeEnd(t.Context(), proofbound.NewWorkBudget(t.Context()), &body.n, &edge.n)
					require.ErrorIs(t, err, ErrUnsupported)
				}
				require.Equal(t, 1, internal)
			}
			motion, err := r3.Translation(r3.Vec{X: 7})
			require.NoError(t, err)
			placed, err := Copy(t.Context(), body, WithCopyPlacement(motion))
			require.NoError(t, err)
			_, _, _, err = revolveplan.FreeEnd(t.Context(), proofbound.NewWorkBudget(t.Context()), &placed.n, &ends[0].n)
			require.ErrorIs(t, err, ErrUnsupported)
			copiedEnds := 0
			for _, edge := range placed.Edges() {
				_, _, _, err := revolveplan.FreeEnd(t.Context(), proofbound.NewWorkBudget(t.Context()), &placed.n, &edge.n)
				if err == nil {
					copiedEnds++
				}
			}
			require.Equal(t, 2, copiedEnds)
		})
	}
}
