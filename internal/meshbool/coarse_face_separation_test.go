package meshbool

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// A coarse face-wide reading can bring separated finite facets into the
// broad contact gather. The prepared patches' own bounds decide the gap.
func TestCoarseFacesSeparatedByFacetBounds(t *testing.T) {
	for _, tc := range []struct {
		name string
		gap  float64
		away bool
	}{{"clear", 0.0625, true}, {"bounds touch", 0.03125, false}, {"held contact", 0, false}} {
		t.Run(tc.name, func(t *testing.T) {
			a := exactBoxMesh(t, r3.Vec{}, r3.NewVec(1, 1, 1))
			b := exactBoxMesh(t, r3.NewVec(1+tc.gap, 0, 0), r3.NewVec(2+tc.gap, 1, 1))
			wide := [6]float64{.125, .125, .125, .125, .125, .125}
			ma, mb := boxTessellation(wide), boxTessellation(wide)
			for _, m := range []*BoolMesh{a, b} {
				m.Gate, m.PairChordLimit = .0078125, .0078125
				for i := range m.Tris {
					m.FacetBound[i], m.SlabBound[i] = .015625, .015625
				}
			}
			err := RefuseUndecidableProximity(t.Context(), ma, mb, a, b, NewContactMemo(a, b), nil, nil)
			if tc.away {
				if err != nil {
					t.Fatal(err)
				}
				return
			}
			if _, ok := errors.AsType[*CoarseHeldContactError](err); !ok {
				t.Fatalf("expected coarse contact; got %v", err)
			}
		})
	}
}
