package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// separateBoss is one radius-6, 8 mm round boss of a separate-bosses case: its
// centre and the level of its free cap, 18 on the plate's top or -8 under its
// bottom.
type separateBoss struct {
	x, y, cap float64
}

// separateBossesCase joins a 100 × 60 × 10 plate with radius-6 round bosses
// that stand apart from each other.
type separateBossesCase struct {
	name   string
	bosses []separateBoss
	join   func(t *testing.T, doc *decad.Document) *decad.Body
	// faces is the result's face count when it differs from the plate's six
	// and a wall and a cap per boss; overhang is the number of floors facing
	// down at z = 10 beside the plate's top.
	faces, overhang int
}

// onTop is a boss standing on the plate's top at (x, 30).
func onTop(x float64) separateBoss { return separateBoss{x: x, y: 30, cap: 18} }

func (b separateBoss) prism(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	z0 := 10.0
	if b.cap < 0 {
		z0 = b.cap
	}
	return pinPrism(t, doc, b.x, b.y, 6, z0, 8)
}

// unionOf joins the plate and the bosses through one Union, in the order the
// plate first and the bosses as listed.
func unionOf(bosses ...separateBoss) func(t *testing.T, doc *decad.Document) *decad.Body {
	return func(t *testing.T, doc *decad.Document) *decad.Body {
		t.Helper()
		plate := boxBody(t, doc, 0, 0, 100, 60, 10)
		tools := make([]*decad.Body, len(bosses))
		for i, b := range bosses {
			tools[i] = b.prism(t, doc)
		}
		joined, err := decad.Union(t.Context(), plate, tools...)
		require.NoError(t, err)
		return joined
	}
}

func separateBossesCases() []separateBossesCase {
	three := []separateBoss{onTop(20), onTop(50), onTop(80)}
	return []separateBossesCase{
		{name: "PatternJoin of three", bosses: three, join: func(t *testing.T, doc *decad.Document) *decad.Body {
			t.Helper()
			plate := boxBody(t, doc, 0, 0, 100, 60, 10)
			joined, err := decad.PatternJoin(t.Context(), plate, onTop(20).prism(t, doc),
				decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(30), Count: 3})
			require.NoError(t, err)
			return joined
		}},
		{name: "Union of three", bosses: three, join: unionOf(three...)},
		{name: "Union of three, last first", bosses: three, join: unionOf(onTop(80), onTop(50), onTop(20))},
		{name: "Union of three, a boss first", bosses: three, join: func(t *testing.T, doc *decad.Document) *decad.Body {
			t.Helper()
			first := onTop(20).prism(t, doc)
			plate := boxBody(t, doc, 0, 0, 100, 60, 10)
			joined, err := decad.Union(t.Context(), first, plate, onTop(50).prism(t, doc), onTop(80).prism(t, doc))
			require.NoError(t, err)
			return joined
		}},
		{name: "Union of two", bosses: three[:2], join: unionOf(three[:2]...)},
		{name: "Union of four", bosses: []separateBoss{onTop(15), onTop(35), onTop(55), onTop(75)},
			join: unionOf(onTop(15), onTop(35), onTop(55), onTop(75))},
		{name: "Union with a boss under the plate", bosses: []separateBoss{onTop(20), {x: 50, y: 30, cap: -8}, onTop(80)},
			join: unionOf(onTop(20), separateBoss{x: 50, y: 30, cap: -8}, onTop(80))},
		// The last boss overhangs the plate's edge x = 100 by 3 mm; it stands on
		// the top, so it adds its whole volume. Its wall splits into the arc
		// standing on the plate and the arc overhanging it, the plate's side
		// x = 100 meets the boss's floor, and the overhang has a floor of its own.
		{name: "Union with a boss over the edge", bosses: []separateBoss{onTop(20), onTop(97)}, join: unionOf(onTop(20), onTop(97)), faces: 12, overhang: 1},
	}
}

// TestStackedUnionSeparateBosses joins a 100 × 60 × 10 plate with round
// bosses of radius 6 and height 8 standing apart from each other, on its top,
// under its bottom or over its edge. A slab both operands reach then holds
// separate circles, which the select-all merge cannot close into one loop, so
// the A1 brep build keeps them as separate regions, with the plate's floor one
// face whose holes are the boss footprints. The union is exact: no shift, the
// volume 100·60·10 + n·π·6²·8 within its bound, one wall per boss, a
// radius-6 Cylinder about the boss's own axis, one cap per boss at its own
// level, and Verify Sound.
func TestStackedUnionSeparateBosses(t *testing.T) {
	t.Parallel()
	for _, tc := range separateBossesCases() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			joined := tc.join(t, doc)
			require.Equal(t, "decad.brepPayload", decadtest.PayloadClass(joined), "the A1 brep builds the union")
			for _, d := range joined.OperationDiagnostics() {
				require.NotEqual(t, decad.OpDiagGapShiftApplied, d.Code, "no boss moved")
			}
			require.Len(t, joined.Lumps(), 1)
			n := len(tc.bosses)
			requireBossesVolume(t, joined, n, jf(0))
			faces := tc.faces
			if faces == 0 {
				faces = 6 + 2*n
			}
			require.Len(t, joined.Faces(), faces, "the plate's six faces, and a wall and a cap per boss")

			axes := map[r3.Vec]struct{}{}
			caps := map[float64]int{}
			for _, f := range joined.Faces() {
				switch s := f.Surface().(type) {
				case decad.Cylinder:
					require.Equal(t, 6.0, s.Radius.Base())
					require.Equal(t, 1.0, math.Abs(s.Axis.Z), "the wall is vertical")
					axes[r3.NewVec(s.Origin.X, s.Origin.Y, 0)] = struct{}{}
				case decad.Plane:
					if math.Abs(s.Frame.N().Z) == 1 {
						caps[s.Frame.Origin().Z]++
					}
				}
			}
			want := map[float64]int{0: 1, 10: 1 + tc.overhang}
			wantAxes := map[r3.Vec]struct{}{}
			for _, b := range tc.bosses {
				wantAxes[r3.NewVec(b.x, b.y, 0)] = struct{}{}
				want[b.cap]++
			}
			require.Equal(t, wantAxes, axes, "each wall stands about its own boss's axis")
			for level, count := range want {
				require.Equal(t, count, caps[level], "horizontal faces at z = %v", level)
			}

			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
		})
	}
}

// requireBossesVolume asserts the body's volume lies within its bound of
// 100·60·10 + n·(π·6²·8 + extra), computed to junctionPrec bits.
func requireBossesVolume(t *testing.T, b *decad.Body, n int, extra *big.Float) {
	t.Helper()
	v, err := b.Volume()
	require.NoError(t, err)
	perBoss := new(big.Float).SetPrec(junctionPrec).Mul(junctionPi(), jf(36*8))
	perBoss.Add(perBoss, extra)
	truth := new(big.Float).SetPrec(junctionPrec).Mul(perBoss, jf(float64(n)))
	truth.Add(truth, jf(60000))
	requireJunctionEncloses(t, "volume", volumeMM(t, v), v.Bound.Base(), truth)
}

// TestStackedUnionSeparateBossesFillet fillets every root and rim of the
// three bosses a PatternJoin stood on the plate, radius 1. A radius-r fillet
// of a circular edge of radius R adds or removes the ring swept by the corner
// region of area A = (1 − π/4)·r², whose centroid lies
// d = r·(10 − 3π)/(3·(4 − π)) from the edge towards the corner. The root adds
// 2π·(R + d)·A and the rim removes 2π·(R − d)·A, so each boss adds 4π·A·d to
// 100·60·10 + π·6²·8.
func TestStackedUnionSeparateBossesFillet(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	joined := separateBossesCases()[0].join(t, doc)
	const r = 1.0
	filleted, err := decad.Fillet(t.Context(), joined, decad.Edges(decad.Circular()).Exactly(6), units.Millimeters(r))
	require.NoError(t, err)
	var tori int
	for _, f := range filleted.Faces() {
		if torus, ok := f.Surface().(decad.Torus); ok {
			require.Equal(t, r, torus.Minor.Base())
			tori++
		}
	}
	require.Equal(t, 6, tori, "each root and rim is blended by a Torus")
	// A = 1 − π/4 and d = (10 − 3π)/(3·(4 − π)) at r = 1; each boss adds 4π·A·d.
	pi := junctionPi()
	area := new(big.Float).SetPrec(junctionPrec).Quo(pi, jf(4))
	area.Sub(jf(1), area)
	num := new(big.Float).SetPrec(junctionPrec).Mul(pi, jf(3))
	num.Sub(jf(10), num)
	den := new(big.Float).SetPrec(junctionPrec).Sub(jf(4), pi)
	den.Mul(den, jf(3))
	d := new(big.Float).SetPrec(junctionPrec).Quo(num, den)
	extra := new(big.Float).SetPrec(junctionPrec).Mul(pi, jf(4))
	extra.Mul(extra, area).Mul(extra, d)
	requireBossesVolume(t, filleted, 3, extra)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

// TestStackedUnionTouchingBosses keeps the shapes whose bosses touch off the
// separate-bosses brep. Two bosses 8 mm apart overlap and merge into one loop,
// a stacked prism whose volume is short of two bosses by the lens they share,
// 8·(72·acos(2/3) − 4·√80). Two bosses 12 mm apart touch at one point, and a
// boss 6 mm in from the plate's edge touches the edge at one point: the brep
// build misses both, and the Union takes the mesh path with a shift, as a
// boss resting on a plate always has.
func TestStackedUnionTouchingBosses(t *testing.T) {
	t.Parallel()
	t.Run("overlapping", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		joined := unionOf(onTop(20), onTop(28))(t, doc)
		require.Equal(t, "decad.stackedPrismPayload", decadtest.PayloadClass(joined))
		lens := 72*math.Acos(2.0/3) - 4*math.Sqrt(80)
		requireVolumeWithin(t, joined, 60000+8*(72*math.Pi-lens))
	})
	for _, tc := range []struct {
		name   string
		bosses []separateBoss
	}{
		{name: "tangent bosses", bosses: []separateBoss{onTop(20), onTop(32)}},
		{name: "tangent to the edge", bosses: []separateBoss{onTop(6), onTop(50)}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			joined := unionOf(tc.bosses...)(t, doc)
			require.True(t, tookMeshPath(joined), "the touching bosses take the mesh path")
			diagnostics := joined.OperationDiagnostics()
			require.NotEmpty(t, diagnostics)
			require.Equal(t, decad.OpDiagGapShiftApplied, diagnostics[len(diagnostics)-1].Code)
		})
	}
}
