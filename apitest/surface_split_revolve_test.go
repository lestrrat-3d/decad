package apitest_test

import (
	"cmp"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md's T184-T186:
// docs/surface-intersection-design.md's PR5, Document.Split over the revolve
// family. Every fixture spins about the sketch plane's own v axis
// (trimRevolveAxis), which is world y, so the meridian's u is the radius ρ
// and its v the axial level.

// splitRevolveSolid revolves the one profile build draws.
func splitRevolveSolid(t *testing.T, doc *decad.Document, build func(*sketch.Sketch), extent decad.AngularExtent) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	build(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := doc.Revolve(s, s.Profiles()[0], trimRevolveAxis(), extent)
	require.NoError(t, err)
	return body
}

// splitRevolveSheet spins the single line (u0,v0)-(u1,v1) as an open
// meridian: a disc or annulus where the line is perpendicular to the axis,
// a cylinder where it is parallel.
func splitRevolveSheet(t *testing.T, doc *decad.Document, u0, v0, u1, v1 float64, extent decad.AngularExtent) *decad.Body {
	t.Helper()
	return splitRevolveSheetAxis(t, doc, u0, v0, u1, v1, extent, trimRevolveAxis())
}

func splitRevolveSheetAxis(t *testing.T, doc *decad.Document, u0, v0, u1, v1 float64, extent decad.AngularExtent, axis decad.Axis) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(u0, v0)
	s.Fix(a)
	b := s.CreatePoint(u1, v1)
	s.Fix(b)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	body, err := doc.RevolveChain(s, s.Chains()[0], axis, extent)
	require.NoError(t, err)
	return body
}

// splitRevolveCylinder is the rectangle r ∈ [0, 4], v ∈ [0, 10]: a solid
// cylinder whose meridian runs along the axis.
func splitRevolveCylinder(s *sketch.Sketch) {
	rect := s.CreateRectangle(0, 0, 4, 10)
	s.Fix(rect.A)
}

// splitRevolveBall is the half disk of radius 5 about the origin, the arc
// from (0, −5) through (5, 0) to (0, 5) closed along the axis: a ball.
func splitRevolveBall(s *sketch.Sketch) {
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	a := s.CreatePoint(0, -5)
	s.Fix(a)
	b := s.CreatePoint(0, 5)
	s.Fix(b)
	s.CreateArc(c, a, b)
	s.CreateLine(b, a)
}

// splitRevolveTorus is the circle of radius 3 about (10, 0): a torus.
func splitRevolveTorus(s *sketch.Sketch) {
	c := s.CreatePoint(10, 0)
	s.Fix(c)
	s.CreateCircle(c, 3)
}

// splitForm is the closed form pi·π + pi2·π² over rationals.
type splitForm struct{ pi, pi2 *big.Rat }

func formPi(n, d int64) splitForm { return splitForm{pi: big.NewRat(n, d), pi2: new(big.Rat)} }

func formPi2(n2, n int64) splitForm {
	return splitForm{pi: big.NewRat(n, 1), pi2: big.NewRat(n2, 1)}
}

// plus is the form's sum with a rational term, for an area holding flat
// faces beside its curved ones.
func (f splitForm) plus(n int64) splitRationalForm { return splitRationalForm{f, big.NewRat(n, 1)} }

type splitRationalForm struct {
	splitForm
	rational *big.Rat
}

// bracket encloses the form over piRefLo/piRefHi, taking each coefficient's
// own sign into account.
func (f splitForm) bracket() (*big.Rat, *big.Rat) {
	pi2Lo := new(big.Rat).Mul(piRefLo, piRefLo)
	pi2Hi := new(big.Rat).Mul(piRefHi, piRefHi)
	term := func(c, lo, hi *big.Rat) (*big.Rat, *big.Rat) {
		a, b := new(big.Rat).Mul(c, lo), new(big.Rat).Mul(c, hi)
		if a.Cmp(b) > 0 {
			a, b = b, a
		}
		return a, b
	}
	l1, h1 := term(f.pi, piRefLo, piRefHi)
	l2, h2 := term(f.pi2, pi2Lo, pi2Hi)
	return l1.Add(l1, l2), h1.Add(h1, h2)
}

func (f splitRationalForm) bracket() (*big.Rat, *big.Rat) {
	lo, hi := f.splitForm.bracket()
	return lo.Add(lo, f.rational), hi.Add(hi, f.rational)
}

type splitBracket interface{ bracket() (*big.Rat, *big.Rat) }

// requireSplitEncloses holds the published interval to the WHOLE reference
// bracket, so an interval that only grazes the reference fails.
func requireSplitEncloses(t *testing.T, what string, m decad.Measurement, want splitBracket) {
	t.Helper()
	lo, hi := want.bracket()
	gotLo, gotHi := splitVolumeInterval(m)
	require.LessOrEqual(t, gotLo.Cmp(lo), 0, "%s: %v ± %v starts above the reference", what, m.Value, m.Bound)
	require.GreaterOrEqual(t, gotHi.Cmp(hi), 0, "%s: %v ± %v ends below the reference", what, m.Value, m.Bound)
}

type splitRevolvePiece struct {
	volume   splitForm
	area     splitBracket
	centroid r3.Vec
}

type splitRevolveCase struct {
	name       string
	build      func(*sketch.Sketch)
	extent     decad.AngularExtent
	tool       [4]float64
	toolExtent decad.AngularExtent
	target     splitForm
	// pieces are listed in ascending centroid y, or in ascending volume where
	// byVolume is set.
	pieces   []splitRevolvePiece
	byVolume bool
}

func splitRevolveCases() []splitRevolveCase {
	quarter := decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along}
	half := decad.AngleExtent{A: units.Degrees(180), Dir: decad.Along}
	return []splitRevolveCase{
		{
			// The disc at v = 6 cuts the axis edge and the outer wall, and
			// leaves cylinders of height 6 and 4: 96π and 64π mm³, walls
			// 48π and 32π plus two 16π discs each.
			name: "cylinder by a disc, full turn", build: splitRevolveCylinder,
			extent: decad.FullRevolution{}, tool: [4]float64{0, 6, 8, 6}, toolExtent: decad.FullRevolution{},
			target: formPi(160, 1),
			pieces: []splitRevolvePiece{
				{volume: formPi(96, 1), area: formPi(80, 1), centroid: r3.NewVec(0, 3, 0)},
				{volume: formPi(64, 1), area: formPi(64, 1), centroid: r3.NewVec(0, 8, 0)},
			},
		},
		{
			// The equatorial disc halves the ball: two hemispheres of
			// 250π/3 mm³ and 75π mm², centroids 3R/8 off the centre.
			name: "ball by an equatorial disc, full turn", build: splitRevolveBall,
			extent: decad.FullRevolution{}, tool: [4]float64{0, 0, 8, 0}, toolExtent: decad.FullRevolution{},
			target: formPi(500, 3),
			pieces: []splitRevolvePiece{
				{volume: formPi(250, 3), area: formPi(75, 1), centroid: r3.NewVec(0, -15.0/8, 0)},
				{volume: formPi(250, 3), area: formPi(75, 1), centroid: r3.NewVec(0, 15.0/8, 0)},
			},
		},
		{
			// The cylinder sheet of radius R = 10 halves the tube's
			// meridian into the inner and outer half disks: Pappus at
			// R ∓ 4r/(3π) gives 90π² ∓ 36π mm³, and the walls at R ∓ 2r/π
			// give 60π² ∓ 36π plus the 120π mm² cut band.
			name: "torus by a coaxial cylinder, full turn", build: splitRevolveTorus,
			extent: decad.FullRevolution{}, tool: [4]float64{10, -5, 10, 5}, toolExtent: decad.FullRevolution{},
			target: formPi2(180, 0), byVolume: true,
			pieces: []splitRevolvePiece{
				{volume: formPi2(90, -36), area: formPi2(60, 84)},
				{volume: formPi2(90, 36), area: formPi2(60, 156)},
			},
		},
		{
			// A quarter turn of the cylinder by a quarter-turn disc: 24π and
			// 16π mm³; each piece adds its two 4 × h angular caps to the
			// quarter of the full turn's area.
			name: "cylinder by a disc, quarter turn", build: splitRevolveCylinder,
			extent: quarter, tool: [4]float64{0, 6, 8, 6}, toolExtent: quarter,
			target: formPi(40, 1),
			pieces: []splitRevolvePiece{
				{volume: formPi(24, 1), area: formPi(20, 1).plus(48)},
				{volume: formPi(16, 1), area: formPi(16, 1).plus(32)},
			},
		},
		{
			// The same quarter turn cut by a FULL-turn disc: S6 needs the
			// tool to cover the target's span, not to equal it.
			name: "cylinder by a full-turn disc, quarter turn", build: splitRevolveCylinder,
			extent: quarter, tool: [4]float64{0, 6, 8, 6}, toolExtent: decad.FullRevolution{},
			target: formPi(40, 1),
			pieces: []splitRevolvePiece{
				{volume: formPi(24, 1), area: formPi(20, 1).plus(48)},
				{volume: formPi(16, 1), area: formPi(16, 1).plus(32)},
			},
		},
		{
			// Half a turn of the torus: half of each full-turn volume, and
			// each piece gains two half-disk caps of 9π/2 mm².
			name: "torus by a coaxial cylinder, half turn", build: splitRevolveTorus,
			extent: half, tool: [4]float64{10, -5, 10, 5}, toolExtent: half,
			target: formPi2(90, 0), byVolume: true,
			pieces: []splitRevolvePiece{
				{volume: formPi2(45, -18), area: formPi2(30, 51)},
				{volume: formPi2(45, 18), area: formPi2(30, 87)},
			},
		},
	}
}

func TestSurfaceSplitRevolveClosedForms(t *testing.T) {
	t.Parallel()
	for _, tc := range splitRevolveCases() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			target := splitRevolveSolid(t, doc, tc.build, tc.extent)
			targetVolume := splitVolume(t, target)
			requireSplitEncloses(t, "target volume", targetVolume, tc.target)
			tool := splitRevolveSheet(t, doc, tc.tool[0], tc.tool[1], tc.tool[2], tc.tool[3], tc.toolExtent)
			require.Equal(t, decad.BodySheet, tool.Kind())

			pieces, err := doc.Split(t.Context(), target, tool)
			require.NoError(t, err)
			require.Len(t, pieces, len(tc.pieces))
			require.Equal(t, pieces, doc.Bodies())

			type measured struct {
				body     *decad.Body
				volume   decad.Measurement
				centroid decad.VecMeasurement
			}
			got := make([]measured, len(pieces))
			for i, piece := range pieces {
				require.Equal(t, decad.BodySolid, piece.Kind())
				require.True(t, piece.IsSolid())
				centroid, err := piece.Centroid()
				require.NoError(t, err)
				got[i] = measured{body: piece, volume: splitVolume(t, piece), centroid: centroid}
			}
			slices.SortFunc(got, func(a, b measured) int {
				if tc.byVolume {
					return cmp.Compare(a.volume.Value.Base(), b.volume.Value.Base())
				}
				return cmp.Compare(a.centroid.Value.Y, b.centroid.Value.Y)
			})
			for i, want := range tc.pieces {
				g := got[i]
				t.Logf("piece %d volume %v ± %v", i, g.volume.Value.Base(), g.volume.Bound.Base())
				require.Equal(t, decad.Approximate, g.volume.Exactness)
				requireSplitEncloses(t, "volume", g.volume, want.volume)
				// The bound must carry the cut parameters' own rounding, not
				// only the build's float noise. Forcing splitRevolve's
				// sectionDelta to 0 leaves every fixture here at most 2.9e-13
				// mm³ of bound, while the charged bounds measure at least
				// 1.1e-11 mm³; 1e-12 sits between the two. Shown-to-fail:
				// that forcing turns this assertion red on every fixture, the
				// ball's narrowed meridian arc included, and leaves every
				// enclosure above green, so this is the leg proving §7.2's
				// charge reaches each piece.
				require.Greater(t, g.volume.Bound.Base(), 1e-12)
				// The value itself, held to the closed form's float image: a
				// bound that encloses a wrong value only by being wide is
				// caught here.
				lo, hi := want.volume.bracket()
				mid, _ := new(big.Rat).Add(lo, hi).Float64()
				require.InEpsilon(t, mid/2, g.volume.Value.Base(), 1e-12)
				area, err := g.body.Area()
				require.NoError(t, err)
				requireSplitEncloses(t, "area", area, want.area)
				if tc.extent == (decad.FullRevolution{}) {
					require.LessOrEqual(t, g.centroid.Value.Sub(want.centroid).Len(), g.centroid.Bound.Base(),
						"centroid %v ± %v misses %v", g.centroid.Value, g.centroid.Bound, want.centroid)
				}
				mesh, err := g.body.Tessellate(t.Context(), units.Millimeters(0.1))
				require.NoError(t, err)
				requireWatertight(t, mesh)
			}

			// The pieces' two intervals sum to an interval enclosing the
			// target's own closed form, and overlapping its own reading.
			lo, hi := tc.target.bracket()
			sumLo, sumHi := new(big.Rat), new(big.Rat)
			for _, g := range got {
				l, h := splitVolumeInterval(g.volume)
				sumLo.Add(sumLo, l)
				sumHi.Add(sumHi, h)
			}
			require.LessOrEqual(t, sumLo.Cmp(lo), 0)
			require.GreaterOrEqual(t, sumHi.Cmp(hi), 0)
			tLo, tHi := splitVolumeInterval(targetVolume)
			require.LessOrEqual(t, sumLo.Cmp(tHi), 0)
			require.GreaterOrEqual(t, sumHi.Cmp(tLo), 0)
		})
	}
}

// splitRevolveOrigins collects every role a body's faces carry.
func splitRevolveOrigins(b *decad.Body) map[decad.FeatureRef]struct{} {
	out := map[decad.FeatureRef]struct{}{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			out[o] = struct{}{}
		}
	}
	return out
}

// TestSurfaceSplitRevolveRolesAndSelectors is T185: each piece is a fresh
// feature whose faces a caller re-selects by geometric predicate or by the
// piece's own cap roles, never by a role the target carried
// (docs/surface-intersection-design.md §9), and the piece order is
// reproduced across a replay.
func TestSurfaceSplitRevolveRolesAndSelectors(t *testing.T) {
	t.Parallel()
	quarter := decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along}
	split := func(t *testing.T, extent decad.AngularExtent) (*decad.Body, []*decad.Body) {
		t.Helper()
		doc := decad.New()
		target := splitRevolveSolid(t, doc, splitRevolveCylinder, extent)
		tool := splitRevolveSheet(t, doc, 0, 6, 8, 6, extent)
		pieces, err := doc.Split(t.Context(), target, tool)
		require.NoError(t, err)
		require.Len(t, pieces, 2)
		// Lower piece first, by centroid level.
		c0, err := pieces[0].Centroid()
		require.NoError(t, err)
		c1, err := pieces[1].Centroid()
		require.NoError(t, err)
		if c0.Value.Y > c1.Value.Y {
			pieces[0], pieces[1] = pieces[1], pieces[0]
		}
		return target, pieces
	}

	t.Run("the cut face selects by predicate", func(t *testing.T) {
		t.Parallel()
		_, pieces := split(t, decad.FullRevolution{})
		up, down := r3.NewVec(0, 1, 0), r3.NewVec(0, -1, 0)
		for i, facing := range []r3.Vec{up, down} {
			faces, err := decad.Faces(decad.Planar(), decad.Facing(facing)).Exactly(1).SelectFaces(pieces[i])
			require.NoError(t, err)
			area, err := faces[0].Area()
			require.NoError(t, err)
			requireSplitEncloses(t, "cut disc area", area, formPi(16, 1))
			for _, e := range faces[0].Edges() {
				require.InDelta(t, 6, e.Start().Position().Value.Y, e.Start().Position().Bound.Base()+1e-12)
			}
			walls, err := decad.Faces(decad.Cylindrical()).Exactly(1).SelectFaces(pieces[i])
			require.NoError(t, err)
			wallArea, err := walls[0].Area()
			require.NoError(t, err)
			requireSplitEncloses(t, "wall area", wallArea, formPi([]int64{48, 32}[i], 1))
		}
	})

	t.Run("roles are fresh and a piece names its own caps", func(t *testing.T) {
		t.Parallel()
		target, pieces := split(t, quarter)
		targetRoles := splitRevolveOrigins(target)
		seen := map[decad.FeatureRef]struct{}{}
		for i, piece := range pieces {
			for role := range splitRevolveOrigins(piece) {
				_, inherited := targetRoles[role]
				require.False(t, inherited, "piece %d carries the target's role %v", i, role)
				_, shared := seen[role]
				require.False(t, shared, "two pieces share the role %v", role)
				seen[role] = struct{}{}
			}
			_, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(target))).SelectFaces(piece)
			require.ErrorIs(t, err, decad.ErrNoMatch)
			// Each angular cap is the 4 × h rectangle of the piece's own
			// meridian.
			for _, ref := range []decad.FeatureRef{decad.CapStart(piece), decad.CapEnd(piece)} {
				caps, err := decad.Faces(decad.FaceCreatedBy(ref)).Exactly(1).SelectFaces(piece)
				require.NoError(t, err)
				area, err := caps[0].Area()
				require.NoError(t, err)
				h := []float64{6, 4}[i]
				require.LessOrEqual(t, absFloat(area.Value.Base()-4*h), area.Bound.Base()+1e-12)
			}
		}
	})

	t.Run("piece order replays", func(t *testing.T) {
		t.Parallel()
		run := func() []decad.Measurement {
			doc := decad.New()
			target := splitRevolveSolid(t, doc, splitRevolveTorus, decad.FullRevolution{})
			tool := splitRevolveSheet(t, doc, 10, -5, 10, 5, decad.FullRevolution{})
			pieces, err := doc.Split(t.Context(), target, tool)
			require.NoError(t, err)
			out := make([]decad.Measurement, len(pieces))
			for i, p := range pieces {
				out[i] = splitVolume(t, p)
			}
			return out
		}
		require.Equal(t, run(), run())
	})
}

// TestSurfaceSplitRevolveRefusals is T186: the revolve pairs Split declines,
// each read by MESSAGE TEXT so the refusal is pinned to its own gate rather
// than only to the sentinel, and each leaving the document unchanged.
func TestSurfaceSplitRevolveRefusals(t *testing.T) {
	t.Parallel()
	quarter := decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along}
	for _, tc := range []struct {
		name     string
		operands func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body)
		sentinel error
		message  string
	}{
		{
			name: "S4 refuses a parallel axis two millimetres away",
			operands: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
				shifted := decad.SketchLine{Start: decad.Point2{U: -2, V: 0}, End: decad.Point2{U: -2, V: 1}}
				return splitRevolveSolid(t, doc, splitRevolveTorus, decad.FullRevolution{}),
					splitRevolveSheetAxis(t, doc, 10, -5, 10, 5, decad.FullRevolution{}, shifted)
			},
			sentinel: decad.ErrUnsupported, message: "do not spin about the same axis, exactly",
		},
		{
			name: "S6 refuses a tool whose span stops inside the target's",
			operands: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
				return splitRevolveSolid(t, doc, splitRevolveCylinder, decad.FullRevolution{}),
					splitRevolveSheet(t, doc, 0, 6, 8, 6, quarter)
			},
			sentinel: decad.ErrUnsupported, message: "does not cover the target's",
		},
		{
			name: "RS7 refuses a tool that separates nothing",
			operands: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
				return splitRevolveSolid(t, doc, splitRevolveCylinder, decad.FullRevolution{}),
					splitRevolveSheet(t, doc, 0, 20, 8, 20, decad.FullRevolution{})
			},
			sentinel: decad.ErrDegenerate, message: "tool separates no part of the target",
		},
		{
			name: "a sheet target refuses",
			operands: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
				return trimRevolveRing(t, doc, 3, 0, 5, 20, true),
					splitRevolveSheet(t, doc, 0, 6, 8, 6, decad.FullRevolution{})
			},
			sentinel: decad.ErrUnsupported, message: "target must be a solid revolve",
		},
		{
			name: "a solid tool refuses",
			operands: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
				return trimRevolveRing(t, doc, 3, 0, 5, 20, false),
					trimRevolveRing(t, doc, 2, 5, 8, 10, false)
			},
			sentinel: decad.ErrUnsupported, message: "tool must be a sheet",
		},
		{
			name: "a piece carrying its own displacement refuses a second split",
			operands: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
				target := splitRevolveSolid(t, doc, splitRevolveCylinder, decad.FullRevolution{})
				tool := splitRevolveSheet(t, doc, 0, 6, 8, 6, decad.FullRevolution{})
				pieces, err := doc.Split(t.Context(), target, tool)
				require.NoError(t, err)
				return pieces[0], splitRevolveSheet(t, doc, 0, 3, 8, 3, decad.FullRevolution{})
			},
			sentinel: decad.ErrUnsupported, message: "section displacement",
		},
		{
			name: "a revolve target and a straight-sweep tool share no generator",
			operands: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
				return splitRevolveSolid(t, doc, splitRevolveCylinder, decad.FullRevolution{}),
					splitRibbon(t, doc, 110)
			},
			sentinel: decad.ErrUnsupported, message: "straight-sweep generator",
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			target, tool := tc.operands(t, doc)
			before := doc.Bodies()
			targetArea, err := target.Area()
			require.NoError(t, err)
			pieces, err := doc.Split(t.Context(), target, tool)
			require.Nil(t, pieces)
			require.ErrorIs(t, err, tc.sentinel)
			require.ErrorContains(t, err, tc.message)
			require.Equal(t, before, doc.Bodies())
			afterArea, err := target.Area()
			require.NoError(t, err)
			require.Equal(t, targetArea, afterArea)
		})
	}
}
