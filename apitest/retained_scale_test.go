package apitest_test

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestRetainedScaledAtKeepsCurvedTags scales the retained-tag fixture of
// agent-docs/faceted-source-retention-design.md (a filleted box drilled by a
// placed tool, a mesh Boolean) and checks the faces keep their analytic kinds
// (agent-docs/retained-modify-reach-design.md §7).
func TestRetainedScaledAtKeepsCurvedTags(t *testing.T) {
	for _, factor := range []float64{2, 1.1} {
		t.Run(units.Scalar(factor).String(), func(t *testing.T) {
			sc := newRetentionScene()
			drilled, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
			require.NoError(t, err)
			t.Logf("before kinds %v", retentionKinds(drilled))
			anchor := r3.NewVec(5, -3, 2)
			scaled, err := decad.ScaledAt(t.Context(), drilled, anchor, units.Scalar(factor))
			require.NoError(t, err)
			kinds := retentionKinds(scaled)
			t.Logf("after kinds %v", kinds)
			require.Zero(t, kinds["other"], "kinds %v", kinds)
			require.Equal(t, 6, kinds["plane"], "kinds %v", kinds)
			require.Equal(t, 5, kinds["cylinder"], "kinds %v", kinds)

			var radii []float64
			for _, f := range scaled.Faces() {
				p := firstVertex(t, f)
				n, err := f.NormalAt(p)
				require.NoError(t, err)
				bound := n.Bound.Base()
				require.Less(t, bound, 1.0, "normal bound at %v", p)
				t.Logf("%T bound %g", f.Surface(), bound)
				switch s := f.Surface().(type) {
				case decad.Plane:
					// Every plane of the fixture is axis aligned and a uniform
					// scale keeps its normal.
					require.LessOrEqual(t, axisDistance(n.Value), bound, "plane normal %v", n.Value)
				case decad.Cylinder:
					radii = append(radii, s.Radius.Base())
					radial := p.Sub(s.Origin)
					radial = radial.Sub(s.Axis.Scale(radial.Dot(s.Axis)))
					want, ok := radial.Normalize()
					require.True(t, ok)
					// The wall of the tool faces into the void.
					gap := min(n.Value.Sub(want).Len(), n.Value.Add(want).Len())
					require.LessOrEqual(t, gap, bound, "cylinder normal %v at %v", n.Value, p)
				}
			}
			slices.Sort(radii)
			require.Len(t, radii, 5)
			for i, source := range []float64{3, 3, 3, 3, 4} {
				// The tag's radius is the source radius times the factor,
				// rounded once.
				require.InEpsilon(t, factor*source, radii[i], 4e-16, "radius %d", i)
			}

			before, err := drilled.Volume()
			require.NoError(t, err)
			after, err := scaled.Volume()
			require.NoError(t, err)
			cube := factor * factor * factor
			slack := after.Bound.Base() + cube*before.Bound.Base() + 4*math.Pow(2, -52)*cube*before.Value.Base()
			require.LessOrEqual(t, math.Abs(after.Value.Base()-cube*before.Value.Base()), slack)
		})
	}
}

// axisDistance is the distance of the unit vector n from the nearest signed
// coordinate axis.
func axisDistance(n r3.Vec) float64 {
	best := math.Inf(1)
	for _, axis := range []r3.Vec{{X: 1}, {Y: 1}, {Z: 1}} {
		best = min(best, n.Sub(axis).Len(), n.Add(axis).Len())
	}
	return best
}
