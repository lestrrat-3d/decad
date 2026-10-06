package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds scenes 2-4 of docs/linkage-check-design.md §11: declared
// joint contacts, the crane, and the near misses that pin the chain travel
// bound's ancestor terms. Scene 1 and the standing tests sit in
// linkage_test.go.

// requireFirstCollisionAbove asserts the onset claims every colliding scene
// shares: every collision sits strictly past star, the first within two grid
// steps of 1/256 above it, every certified interval ends at or before it, and
// the interval holding it is never clear.
func requireFirstCollisionAbove(t *testing.T, report *decad.LinkageReport, star float64, a, b *decad.Body) {
	t.Helper()
	require.Equal(t, decad.Interfering, report.Status)
	require.NotEmpty(t, report.Collisions)
	first := report.Collisions[0]
	require.Same(t, a, first.A)
	require.Same(t, b, first.B)
	require.Greater(t, first.At.Mag(), star)
	require.LessOrEqual(t, first.At.Mag()-star, 2.0/256, `the onset is bracketed to the resolution`)
	require.Less(t, first.Volume.Bound.Mag(), first.Volume.Value.Mag())
	for _, c := range report.Collisions {
		require.Greater(t, c.At.Mag(), star)
	}
	for _, iv := range report.Intervals {
		if iv.Outcome == decad.IntervalClear {
			require.LessOrEqual(t, iv.To.Mag(), star)
		}
		if iv.From.Mag() <= star && star <= iv.To.Mag() {
			require.NotEqual(t, decad.IntervalClear, iv.Outcome, `the interval holding the onset is never clear`)
		}
	}
}

// requireNoFindingNames asserts no diagnostic of the report names the pair.
func requireNoFindingNames(t *testing.T, report *decad.LinkageReport, a, b *decad.Body) {
	t.Helper()
	for _, d := range report.Diagnostics {
		if d.Pair == nil {
			continue
		}
		named := (d.Pair.A == a && d.Pair.B == b) || (d.Pair.A == b && d.Pair.B == a)
		require.Falsef(t, named, `a declared joint contact raised %s`, d.Code)
	}
}

// stackedElbow is scene 2: the upper arm A of scene 1, the forearm B lowered
// to z ∈ [10, 20] so its underside rests on A's cap plane z = 10, and a fence
// x ∈ [0, 10], y ∈ [−40, 40], z ∈ [10.5, 30] rigidly in A's link. The
// shoulder is unlisted and holds 0; the elbow swings B 0° → 180° back over A.
type stackedElbow struct {
	doc             *decad.Document
	upper, forearm  *decad.Body
	fence           *decad.Body
	linkage         *decad.Linkage
	shoulder, elbow *decad.Link
}

func buildStackedElbow(t *testing.T, declare bool) stackedElbow {
	t.Helper()
	e := stackedElbow{doc: decad.New()}
	e.upper = boxBody(t, e.doc, 0, -14, 48, 14, 10)
	e.forearm = boxBodyAtZ(t, e.doc, 48, -14, 96, 14, 10, 10)
	e.fence = boxBodyAtZ(t, e.doc, 0, -40, 10, 40, 10.5, 19.5)
	e.linkage = decad.NewLinkage()
	var err error
	e.shoulder, err = e.linkage.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{e.upper, e.fence})
	require.NoError(t, err)
	e.elbow, err = e.shoulder.Revolute(r3.NewVec(48, 0, 0), zAxis, []*decad.Body{e.forearm})
	require.NoError(t, err)
	if declare {
		require.NoError(t, e.linkage.DeclareJointContact(e.upper, e.forearm))
	}
	return e
}

func (e stackedElbow) drive() decad.Drive {
	return decad.Drive{{Link: e.elbow, From: units.Degrees(0), To: units.Degrees(180)}}
}

// TestVerifyLinkageDeclaredJointContact is scene 2.
//
// B's far corner (96, 14) sits at radius 50 and polar angle atan(7/24) from
// the elbow, and reaches the fence's face x = 10, 38 mm behind the elbow,
// when cos(φ + atan(7/24)) = −38/50: φ* = acos(−19/25) − atan(7/24) ≈ 123.2°,
// s* = φ*/180°. Every other point of B's leading side reaches the face
// later. The declared (A, B) contact touches across A's cap at every pose and
// publishes nothing, while the (B, fence) pair certifies the drive up to the
// contact. Undeclared, the same touching pair holds every interval
// undecided.
//
// A declared pair is still proven for overlap: a 10 mm cube on a prismatic
// joint sliding 0 → 5 mm along X, sunk 1 mm into a static slab, collides at
// both ends with a 10 × 10 × 1 mm³ overlap.
//
// Legs seen to fail when deleted, each in the subtest named: keeping a
// declared pair out of the interval certificate (the fence scene reads no
// interval clear); suppressing a declared pair's box-proven, undecided-pair
// and untransferred-overlap findings, and its touching row (the fence scene
// and the touching and undecided scene raise them); suppressing its sheet
// finding (the sheet scene); keeping its measured gap out of findings, the
// margin and the reading (the measured-gap scene violates 3 mm); and still
// publishing its transferred collision (the sunk cube reports none).
func TestVerifyLinkageDeclaredJointContact(t *testing.T) {
	t.Parallel()
	phiStar := math.Acos(-19.0/25) - math.Atan(7.0/24)
	sStar := phiStar / math.Pi
	t.Run("the declared elbow lets the fence collision be found", func(t *testing.T) {
		t.Parallel()
		e := buildStackedElbow(t, true)
		report := verifyLinkage(t, e.doc, e.linkage, e.drive(), decad.WithResolution(units.Scalar(1.0/256)))
		require.Equal(t, []decad.DiagnosticPair{{A: e.upper, B: e.forearm}}, report.JointContacts)
		requireFirstCollisionAbove(t, report, sStar, e.fence, e.forearm)
		// At the first collision the leading corner pokes depth d past the
		// fence's face: a triangular prism with legs d/(−cos φ) and d/sin φ
		// over the 9.5 mm the two share in z.
		first := report.Collisions[0]
		phi := math.Pi * first.At.Mag()
		depth := 10 - (48 + 48*math.Cos(phi) - 14*math.Sin(phi))
		require.InDelta(t, 9.5*depth*depth/(2*-math.Cos(phi)*math.Sin(phi)), first.Volume.Value.Mag(), 1e-6)
		requireNoFindingNames(t, report, e.upper, e.forearm)
		certified := 0
		for _, iv := range report.Intervals {
			if iv.Outcome == decad.IntervalClear {
				certified++
			}
		}
		require.Positive(t, certified, `the declared pair holds no interval undecided`)
		for _, c := range report.Collisions {
			require.NotSame(t, e.upper, c.A, `a touching declared pair is no collision`)
		}
	})
	t.Run("undeclared, the touching elbow holds every interval undecided", func(t *testing.T) {
		t.Parallel()
		e := buildStackedElbow(t, false)
		report := verifyLinkage(t, e.doc, e.linkage, e.drive(), decad.WithResolution(units.Scalar(1.0/16)))
		require.Empty(t, report.JointContacts)
		require.NotEqual(t, decad.Sound, report.Status)
		for _, iv := range report.Intervals {
			require.NotEqual(t, decad.IntervalClear, iv.Outcome)
		}
		named := false
		for _, d := range report.Diagnostics {
			if d.Pair != nil && d.Pair.A == e.upper && d.Pair.B == e.forearm {
				named = true
			}
		}
		require.True(t, named, `the undeclared touching pair raises its own finding`)
	})
	t.Run("a declared pair's measured gap is a row and nothing else", func(t *testing.T) {
		t.Parallel()
		// The motion arm swings 0° → 90° away from a block in its own
		// layer, 12 mm past its tip at rest. Declared, the pair publishes its
		// gap row at each pose, but the gap enters no interval, no
		// whole-drive reading and no margin, so a 15 mm minimum the gap
		// falls short of raises nothing and the endpoints settle the drive.
		doc := decad.New()
		arm := motionArm(t, doc)
		block := boxBody(t, doc, 60, -5, 70, 5, 10)
		l := decad.NewLinkage()
		swing, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{arm})
		require.NoError(t, err)
		require.NoError(t, l.DeclareJointContact(arm, block))
		report := verifyLinkage(t, doc, l, decad.Drive{{Link: swing, From: units.Degrees(0), To: units.Degrees(90)}},
			decad.WithMinClearance(units.Millimeters(15)))
		require.Equal(t, decad.Sound, report.Status)
		require.Empty(t, report.Diagnostics)
		require.Equal(t, decad.AssessmentMet, report.Assessment)
		require.Nil(t, report.Clearance, `a declared pair enters no whole-drive reading`)
		require.Len(t, report.Poses, 2)
		for _, p := range report.Poses {
			require.Len(t, p.Clearances, 1)
		}
		require.Equal(t, 12.0, report.Poses[0].Clearances[0].Gap.Value.Mag())
	})
	t.Run("a declared pair's touching or undecided outcome publishes nothing", func(t *testing.T) {
		t.Parallel()
		// A block resting on a slab's top face touches it, and a block sunk
		// 1 mm into a slab across their shared face plane x = 0 overlaps it
		// by a volume the read-only proof cannot measure. Each slides along
		// +Y, keeping that plane shared. Declared, neither pair publishes a
		// row or a finding.
		for _, z0 := range []float64{0, -1} {
			doc := decad.New()
			block := boxBodyAtZ(t, doc, 0, -5, 10, 5, z0, 10)
			slab := boxBodyAtZ(t, doc, 0, -20, 40, 20, -10, 10)
			l := decad.NewLinkage()
			slide, err := l.Ground().Prismatic(r3.NewVec(0, 1, 0), []*decad.Body{block})
			require.NoError(t, err)
			require.NoError(t, l.DeclareJointContact(block, slab))
			report := verifyLinkage(t, doc, l, decad.Drive{{Link: slide, From: units.Millimeters(0), To: units.Millimeters(4)}})
			require.Equal(t, decad.Sound, report.Status, "z0 = %v", z0)
			require.Empty(t, report.Diagnostics)
			require.Empty(t, report.Collisions)
			for _, p := range report.Poses {
				require.Empty(t, p.Clearances)
				require.Empty(t, p.Interferences)
			}
		}
	})
	t.Run("a declared sheet pair publishes nothing", func(t *testing.T) {
		t.Parallel()
		// A block sliding inside a rectangular sheet's walls forms a pair no
		// kernel measures; undeclared it raises DiagUnsupportedPairSheet at
		// every pose, declared it raises nothing.
		for _, declare := range []bool{false, true} {
			doc := decad.New()
			sheet := trimRectSheet(t, doc)
			block := boxBody(t, doc, 40, 20, 50, 30, 10)
			l := decad.NewLinkage()
			slide, err := l.Ground().Prismatic(r3.NewVec(0, 1, 0), []*decad.Body{block})
			require.NoError(t, err)
			if declare {
				require.NoError(t, l.DeclareJointContact(block, sheet))
			}
			report := verifyLinkage(t, doc, l, decad.Drive{{Link: slide, From: units.Millimeters(0), To: units.Millimeters(4)}})
			sheets := 0
			for _, d := range report.Diagnostics {
				if d.Code == decad.DiagUnsupportedPairSheet {
					sheets++
				}
			}
			if declare {
				require.Zero(t, sheets)
				require.Equal(t, decad.Sound, report.Status)
				continue
			}
			require.Equal(t, len(report.Poses), sheets)
		}
	})
	t.Run("a declared pair is still proven for overlap", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		cube := boxBodyAtZ(t, doc, 0, -5, 10, 5, -1, 10)
		slab := boxBodyAtZ(t, doc, -20, -20, 40, 20, -10, 10)
		l := decad.NewLinkage()
		slide, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{cube})
		require.NoError(t, err)
		require.NoError(t, l.DeclareJointContact(slab, cube))
		report := verifyLinkage(t, doc, l, decad.Drive{{Link: slide, From: units.Millimeters(0), To: units.Millimeters(5)}},
			decad.WithResolution(units.Scalar(1.0/256)))
		require.Equal(t, decad.Interfering, report.Status)
		require.Equal(t, []decad.DiagnosticPair{{A: slab, B: cube}}, report.JointContacts)
		require.Len(t, report.Collisions, 2)
		require.Equal(t, units.Scalar(0), report.Collisions[0].At)
		require.Equal(t, units.Scalar(1), report.Collisions[1].At)
		for _, c := range report.Collisions {
			require.Same(t, cube, c.A)
			require.Same(t, slab, c.B)
			require.InDelta(t, 100, c.Volume.Value.Mag(), 1e-6)
		}
	})
}

// TestVerifyLinkageCrane is scene 3: a mast x, y ∈ [−5, 5], z ∈ [0, 38]
// turning 0° → 90° about Z, and a boom x ∈ [10, 60], y ∈ [−5, 5],
// z ∈ [40, 50] extending 0 → 30 mm along the mast's +X. The boom's tip
// corner (60 + 30s, 5) in the mast's frame, turned by θ = 90°·s, stands at
// y = (60 + 30s)·sin θ + 5·cos θ, increasing in s, and reaches the wall's
// face y = 60 at s*, its one root in [0, 1], bracketed here to 1e-12 by
// bisection of the closed form. The boom slides along X, perpendicular to Z,
// so the mast's z-extent [0, 38] and the boom's [40, 50] hold at every s and
// the layer exclusion (§5.7) settles the pair.
//
// The prismatic term is pinned with the mast unlisted: a block on the boom's
// joint passes straight through a 2 mm pin, 4 mm ahead of it at rest and
// 14 mm behind it at the end, and the endpoints alone never read that clear.
//
// Legs seen to fail when deleted: the prismatic joint's |Δq| term in τ (the
// pin's interval certifies clear).
func TestVerifyLinkageCrane(t *testing.T) {
	t.Parallel()
	t.Run("the boom reaches the wall", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		mast := boxBody(t, doc, -5, -5, 5, 5, 38)
		boom := boxBodyAtZ(t, doc, 10, -5, 60, 5, 40, 10)
		wall := boxBodyAtZ(t, doc, -100, 60, 150, 80, 30, 70)
		l := decad.NewLinkage()
		turn, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{mast})
		require.NoError(t, err)
		extend, err := turn.Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{boom})
		require.NoError(t, err)
		report := verifyLinkage(t, doc, l, decad.Drive{
			{Link: turn, From: units.Degrees(0), To: units.Degrees(90)},
			{Link: extend, From: units.Millimeters(0), To: units.Millimeters(30)},
		}, decad.WithResolution(units.Scalar(1.0/256)))

		height := func(s float64) float64 {
			th := s * math.Pi / 2
			return (60+30*s)*math.Sin(th) + 5*math.Cos(th) - 60
		}
		lo, hi := 0.0, 1.0
		for hi-lo > 1e-12 {
			mid := (lo + hi) / 2
			if height(mid) < 0 {
				lo = mid
				continue
			}
			hi = mid
		}
		requireFirstCollisionAbove(t, report, hi, boom, wall)
		// At the first collision the tip corner pokes depth d past y = 60: a
		// triangular prism with legs d/sin θ and d/cos θ over the boom's
		// 10 mm height.
		first := report.Collisions[0]
		depth := height(first.At.Mag())
		th := first.At.Mag() * math.Pi / 2
		require.InDelta(t, 10*depth*depth/(2*math.Sin(th)*math.Cos(th)), first.Volume.Value.Mag(), 1e-6)
		for _, p := range report.Poses {
			for _, row := range p.Clearances {
				require.False(t, row.A == mast && row.B == boom, `the layer exclusion settles the mast and boom`)
			}
		}
	})
	t.Run("a pin between samples is never certified clear from the endpoints", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		mast := boxBody(t, doc, -5, -5, 5, 5, 38)
		block := boxBodyAtZ(t, doc, 10, -5, 20, 5, 40, 10)
		boxBodyAtZ(t, doc, 24, -1, 26, 1, 44, 2)
		l := decad.NewLinkage()
		turn, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{mast})
		require.NoError(t, err)
		extend, err := turn.Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{block})
		require.NoError(t, err)
		report := verifyLinkage(t, doc, l, decad.Drive{{Link: extend, From: units.Millimeters(0), To: units.Millimeters(30)}},
			decad.WithResolution(units.Scalar(1)))
		require.Len(t, report.Poses, 2)
		require.Empty(t, report.Collisions)
		require.Equal(t, decad.IntervalUndecided, report.Intervals[0].Outcome)
		require.Equal(t, decad.Suspect, report.Status)
	})
}

// nearMissPin is motion §9 test 3's 0.8 mm pin, z ∈ [4.6, 5.4], centred at
// radius r and polar angle α = 90·31/64°.
func nearMissPin(t *testing.T, doc *decad.Document, r float64) {
	t.Helper()
	a := 90.0 * 31 / 64 * math.Pi / 180
	cx, cy := r*math.Cos(a), r*math.Sin(a)
	boxBodyAtZ(t, doc, cx-0.4, cy-0.4, cx+0.4, cy+0.4, 4.6, 0.8)
}

// TestVerifyLinkageNearMissBetweenSamples is scene 4: motion §9 test 3's
// blade and pin in two chains, each under a hub x, y ∈ [−5, 5],
// z ∈ [−10, −5] turning 0° → 90° about Z. The pin's angle is the fraction
// s_α = 31/64. At WithResolution(1/30) refinement stops at width 1/32, whose
// grid points 15/32 and 16/32 sit 1.40625° from α and clear of the pin; at
// WithResolution(1/900) the grid reaches s_α.
//
//   - 4a: the blade x ∈ [0, 50] slides 0 → 0.1 mm along +X on its own
//     prismatic joint, so its swing about Z is the hub joint's alone. The
//     contact window is |θ − α| ≤ 1.247°, ±0.013856 in s.
//   - 4b: the blade x ∈ [50, 100] hangs from an elbow at (100, 0, 0) held at
//     180°, so it points outward and its tip is 150 mm from the hub's axis;
//     the pin sits at radius 149, with a window of ±0.41° (±0.41/90 in s).
//     The ball under the elbow reads ρ_12 = 100 + √(50² + 0.5² + 10²) ≈ 151;
//     over a width-1/32 interval that travel, ≈ 7.4 mm, exceeds the two gaps'
//     sum, 5.18 mm, while ρ_12 = 100 gives 4.9 mm and certifies it.
//
// Legs seen to fail when deleted: the ancestor joint's term in a link's τ
// (4a's blade travels 0.1·Δs and the coarse interval around the pin
// certifies); and the revolute ball's radius in ρ_{ik} (4b reads
// ρ_12 = 100 and certifies it).
func TestVerifyLinkageNearMissBetweenSamples(t *testing.T) {
	t.Parallel()
	alpha := 31.0 / 64
	cases := []struct {
		name   string
		build  func(t *testing.T) (*decad.Document, *decad.Linkage, decad.Drive)
		window float64
	}{
		{"4a the whole swing on the ancestor joint", func(t *testing.T) (*decad.Document, *decad.Linkage, decad.Drive) {
			doc := decad.New()
			hub := boxBodyAtZ(t, doc, -5, -5, 5, 5, -10, 5)
			blade := boxBody(t, doc, 0, -0.5, 50, 0.5, 10)
			nearMissPin(t, doc, 49)
			l := decad.NewLinkage()
			swing, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{hub})
			require.NoError(t, err)
			slide, err := swing.Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{blade})
			require.NoError(t, err)
			return doc, l, decad.Drive{
				{Link: swing, From: units.Degrees(0), To: units.Degrees(90)},
				{Link: slide, From: units.Millimeters(0), To: units.Millimeters(0.1)},
			}
		}, 0.013856},
		{"4b the ball under a held elbow", func(t *testing.T) (*decad.Document, *decad.Linkage, decad.Drive) {
			doc := decad.New()
			hub := boxBodyAtZ(t, doc, -5, -5, 5, 5, -10, 5)
			blade := boxBody(t, doc, 50, -0.5, 100, 0.5, 10)
			nearMissPin(t, doc, 149)
			l := decad.NewLinkage()
			swing, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{hub})
			require.NoError(t, err)
			elbow, err := swing.Revolute(r3.NewVec(100, 0, 0), zAxis, []*decad.Body{blade})
			require.NoError(t, err)
			return doc, l, decad.Drive{
				{Link: swing, From: units.Degrees(0), To: units.Degrees(90)},
				{Link: elbow, From: units.Degrees(180), To: units.Degrees(180)},
			}
		}, 0.41 / 90},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			t.Run("a coarse resolution never reads the pass clear", func(t *testing.T) {
				t.Parallel()
				doc, l, drive := tc.build(t)
				report := verifyLinkage(t, doc, l, drive, decad.WithResolution(units.Scalar(1.0/30)))
				require.Empty(t, report.Collisions)
				require.Equal(t, decad.Suspect, report.Status)
				found := false
				for _, iv := range report.Intervals {
					if iv.From.Mag() <= alpha && alpha <= iv.To.Mag() {
						found = true
						require.Equal(t, decad.IntervalUndecided, iv.Outcome)
					}
				}
				require.True(t, found)
			})
			t.Run("a fine resolution finds the pin", func(t *testing.T) {
				t.Parallel()
				doc, l, drive := tc.build(t)
				report := verifyLinkage(t, doc, l, drive, decad.WithResolution(units.Scalar(1.0/900)))
				require.Equal(t, decad.Interfering, report.Status)
				require.NotEmpty(t, report.Collisions)
				require.InDelta(t, alpha, report.Collisions[0].At.Mag(), tc.window)
			})
		})
	}
}
