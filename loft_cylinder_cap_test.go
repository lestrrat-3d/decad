package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/stretchr/testify/require"
)

// TestLoftCylinderEdgesMeetXPAgreesWithRational drives the float-filtered
// edge contact predicate and the rational one over segment pairs that touch
// at an endpoint, cross, overlap collinearly, and stay disjoint with
// overlapping bounding boxes.
func TestLoftCylinderEdgesMeetXPAgreesWithRational(t *testing.T) {
	t.Parallel()
	pt := func(x, y float64) loftCylinderXY {
		return loftCylinderXY{new(big.Rat).SetFloat64(x), new(big.Rat).SetFloat64(y)}
	}
	for _, tc := range []struct {
		name       string
		a, b, c, d loftCylinderXY
		want       bool
	}{
		{"touch at endpoint", pt(0, 0), pt(2, 0), pt(2, 0), pt(3, 3), true},
		{"endpoint on interior", pt(0, 0), pt(4, 0), pt(2, 0), pt(2, 3), true},
		{"cross", pt(0, 0), pt(2, 2), pt(0, 2), pt(2, 0), true},
		{"collinear overlap", pt(0, 0), pt(3, 0), pt(2, 0), pt(5, 0), true},
		{"collinear disjoint", pt(0, 0), pt(1, 0), pt(2, 0), pt(5, 0), false},
		{"overlapping boxes disjoint", pt(0, 0), pt(4, 4), pt(0, 1), pt(2, 3.5), false},
		{"near miss by one ulp", pt(0, 0), pt(1, 1), pt(1.0000000000000002, 1), pt(3, 0), false},
		{"parallel apart", pt(0, 0), pt(2, 2), pt(0, 1), pt(2, 3), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t, tc.want, loftCylinderEdgesMeet(tc.a, tc.b, tc.c, tc.d))
			xp := func(p loftCylinderXY) meshbool.Xp2 { return meshbool.NewXP2(p.x, p.y) }
			require.Equal(t, tc.want, loftCylinderEdgesMeetXP(xp(tc.a), xp(tc.b), xp(tc.c), xp(tc.d)))
			// A point without a finite float image takes the rational body.
			huge := new(big.Rat).SetFrac(new(big.Int).Lsh(big.NewInt(1), 2000), big.NewInt(1))
			far := loftCylinderXY{huge, huge}
			require.Equal(t, loftCylinderEdgesMeet(tc.a, tc.b, tc.c, far),
				loftCylinderEdgesMeetXP(xp(tc.a), xp(tc.b), xp(tc.c), xp(far)))
		})
	}
}
