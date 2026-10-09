package decad

import (
	"math"
	"math/big"
	"slices"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin route P of docs/brep-modify-design.md (§4, Table BB
// rows BB1–BB3): a brep receiver that reads as a prism along a reference axis
// takes the shipped prism op, built through the public Cut so the records are
// the real ones. Every volume is asserted against its closed form, and every
// refusal leaves the receiver live and the document unchanged.

// internalSquareHoleBox is §9's B1: the 40×20×20 box cut by the 10×10 square
// hole along y at x∈[15,25], z∈[5,15].
func internalSquareHoleBox(t *testing.T) (*Document, *Body) {
	t.Helper()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XZ(), -10)
	require.NoError(t, err)
	tool := internalClassBTool(t, doc, w, plane, 11, func(s *sketch.Sketch) {
		r := s.CreateRectangle(15, 5, 25, 15)
		s.Fix(r.A)
	})
	result, err := Cut(t.Context(), box, tool)
	require.NoError(t, err)
	_, ok := result.payload.(brepPayload)
	require.True(t, ok, "the public Cut builds B1 as a brep body, got %T", result.payload)
	return doc, result
}

// internalCrossDrilledBrepRim returns S1's circular rim edge centred on y.
func internalCrossDrilledBrepRim(t *testing.T, body *Body, center r3.Vec) *Edge {
	t.Helper()
	var rim *Edge
	for _, e := range body.Edges() {
		if c, ok := e.curve.(Circle3); ok && c.Center == center {
			require.Nil(t, rim, "one rim edge is centred at %v", center)
			rim = e
		}
	}
	require.NotNil(t, rim, "a rim edge is centred at %v", center)
	return rim
}

// requireRefusesUnchanged requires err to be ErrUnsupported naming every one
// of want, with body still live and its document's body set unchanged.
func requireRefusesUnchanged(t *testing.T, body *Body, before []*Body, err error, want ...string) {
	t.Helper()
	require.ErrorIs(t, err, ErrUnsupported)
	for _, w := range want {
		require.ErrorContains(t, err, w)
	}
	require.NoError(t, body.doc.requireLive(body))
	require.Equal(t, before, body.doc.Bodies())
}

// TestBrepModifyRoutePFilletsS1LateralEdges pins BB1: S1 reads as the
// plate-with-a-hole prism along y, so its four outer edges along y are that
// prism's lateral edges, and the fillet is the prism's own. Shown to fail
// with brepPrismRoute returning an empty route (the call refused SX16).
func TestBrepModifyRoutePFilletsS1LateralEdges(t *testing.T) {
	t.Parallel()
	doc, s1 := internalCrossDrilled(t)
	before := len(doc.Bodies())
	result, err := s1.Fillet(t.Context(), Edges(ParallelTo(r3.NewVec(0, 1, 0))).Exactly(4), units.Millimeters(2))
	require.NoError(t, err)
	_, ok := result.payload.(prismPayload)
	require.True(t, ok, "route P builds a prism, got %T", result.payload)
	require.Len(t, result.Faces(), 11)
	// The section is 800 less four corners of 4 − π and the Ø6 hole, swept
	// over 20: 20·(784 − 5π).
	lo, hi := piEnclosed(big.NewRat(15680, 1), big.NewRat(-100, 1))
	requireCoversInterval(t, result.volume, lo, hi)
	roles := facesByRole(result)
	fillets := 0
	for role := range roles {
		if strings.HasPrefix(role, "fillet(0,") {
			fillets++
		}
	}
	require.Equal(t, 4, fillets)
	require.Contains(t, roles, roleCapStart)
	require.Contains(t, roles, roleCapEnd)
	require.ErrorIs(t, doc.requireLive(s1), ErrRetiredBody, "the receiver is retired")
	require.Len(t, doc.Bodies(), before)
}

// TestBrepModifyRoutePChamfersS1HoleRim pins BB2: S1's hole rim at y = 0 is
// one complete loop of the recognised prism's start cap, so the chamfer is
// the prism's cap-loop chamfer. The removed ring is the triangle of legs 1
// revolved at radius 3 + 1/3, 10π/3. Shown to fail with brepPrismRead.caps
// naming no cap face (the rim classified as no cap edge, SX16).
func TestBrepModifyRoutePChamfersS1HoleRim(t *testing.T) {
	t.Parallel()
	doc, s1 := internalCrossDrilled(t)
	rim := internalCrossDrilledBrepRim(t, s1, r3.NewVec(20, 0, 10))
	result, err := s1.Chamfer(t.Context(), Edges(Circular(), EndpointAt(rim.start.position)).Exactly(1), units.Millimeters(1))
	require.NoError(t, err)
	_, ok := result.payload.(capBlendPayload)
	require.True(t, ok, "route P builds a cap-loop chamfer, got %T", result.payload)
	lo, hi := piEnclosed(big.NewRat(16000, 1), big.NewRat(-550, 3))
	requireCoversInterval(t, result.volume, lo, hi)
	var cones []*Face
	for _, f := range result.Faces() {
		if _, ok := f.Surface().(Cone); ok {
			cones = append(cones, f)
		}
	}
	require.Len(t, cones, 1)

	// Pulling along +y hooks the countersink alone; the concave-radius survey
	// reads the whole-turn band as modify-reach DX8 does, undecided.
	rep, err := doc.Verify(t.Context(), WithPullDirection(r3.NewVec(0, 1, 0)), WithConcaveRadius())
	require.NoError(t, err)
	br, err := rep.ForBody(result)
	require.NoError(t, err)
	require.Equal(t, CoverageComplete, br.Undercut.Coverage)
	require.Equal(t, cones, br.Undercut.Faces)
	require.Equal(t, ScalarUndecided, br.ConcaveRadius.Outcome)
	require.Len(t, br.ConcaveRadius.Diagnostics, 1)
	require.Equal(t, DiagUndecidedMinRadius, br.ConcaveRadius.Diagnostics[0].Code)
}

// TestBrepModifyRoutePShellsS1 pins BB3: removing S1's y = 20 face shells
// the recognised prism into a cup open at y = 20, and removing its x = 0
// face, a side wall of every prism it reads as, falls past route P to route
// S, which reads S1 as the box along z and opens that wall
// (docs/modify-general-design.md §3.2). The cup is 20·(800 − 9π) less the
// cavity 18·(576 − 25π). The opened box is 16000 − 180π less the cavity
// [0, 38] × [2, 18] over z ∈ [2, 18] less the hole dilated to radius 5 over
// y ∈ [2, 18], 608·16 − 400π: 6272 + 220π, and the rim at x = 0 faces −x.
// Shown to fail with
// brepPrismRead.caps swapping the start and end caps (the cup opened at
// y = 0, its +y faces all at y = 20) and with the rim of a removed wall along
// z stated in the frame facing into the material (the rim then faced +x; the
// volume does not see it, since a face at x = 0 adds no flux).
func TestBrepModifyRoutePShellsS1(t *testing.T) {
	t.Parallel()
	y := r3.NewVec(0, 1, 0)
	t.Run("cup", func(t *testing.T) {
		t.Parallel()
		_, s1 := internalCrossDrilled(t)
		result, err := s1.Shell(t.Context(), Faces(Facing(y)).Exactly(1), units.Millimeters(2))
		require.NoError(t, err)
		_, ok := result.payload.(cupPayload)
		require.True(t, ok, "route P builds a cup, got %T", result.payload)
		lo, hi := piEnclosed(big.NewRat(5632, 1), big.NewRat(270, 1))
		requireCoversInterval(t, result.volume, lo, hi)
		// Open at y = 20: the outer rim and the hole's post top there, and
		// the cavity floor at y = 2, face +y.
		up, err := Faces(Facing(y)).SelectFaces(result)
		require.NoError(t, err)
		levels := make([]float64, 0, len(up))
		for _, f := range up {
			levels = append(levels, f.Loops()[0].CoEdges()[0].Start().Position().Value.Y)
		}
		slices.Sort(levels)
		require.Equal(t, []float64{2, 20, 20}, levels)
	})
	t.Run("side wall", func(t *testing.T) {
		t.Parallel()
		_, s1 := internalCrossDrilled(t)
		result, bp := requireThroughShell(t, s1, Faces(Facing(r3.NewVec(-1, 0, 0))).Exactly(1), units.Millimeters(2),
			big.NewRat(6272, 1), big.NewRat(220, 1))
		require.Len(t, bp.faces, 13)
		// The rim at x = 0 faces out of the material, as the wall did.
		shellFaceAt(t, result, r3.NewVec(-1, 0, 0), 0)
	})
}

// TestBrepModifyRoutePChamfersB1SquareHole pins BB1 over hole corners: B1's
// four hole edges along y are concave corners of the recognised section, so
// each 2 mm chamfer fills a 2 mm² triangle over 20 mm. Every coordinate is
// recorded, so the volume is Exact. Shown to fail with brepPrismRoute
// returning an empty route (SX16).
func TestBrepModifyRoutePChamfersB1SquareHole(t *testing.T) {
	t.Parallel()
	_, b1 := internalSquareHoleBox(t)
	result, err := b1.Chamfer(t.Context(), Edges(ParallelTo(r3.NewVec(0, 1, 0)), Concave()).Exactly(4), units.Millimeters(2))
	require.NoError(t, err)
	_, ok := result.payload.(prismPayload)
	require.True(t, ok, "route P builds a prism, got %T", result.payload)
	require.Equal(t, Exact, result.volume.Exactness)
	require.Equal(t, 14160.0, result.volume.Value.Base())
	require.Zero(t, result.volume.Bound.Base())
}

// TestBrepModifyRecognisesThePrism pins §4.1. S1 reads as a prism along y
// alone, with F the axis frame (U = z, V = x, N = y) since neither cap's
// frame faces +y, and S top's region re-expressed into F and re-wound, bit
// for bit. P5 refuses a hand-built S1 whose hole wall is walked against its
// material, and one whose z = 0 face faces into the material. Shown to fail
// with brepgeom.WalkKeyOf ignoring the circular sense (the reversed wall then
// read), with brepgeom.PrismRect.Matches ignoring the normal (the inward-facing
// rectangle then read), and with the two rectangle readers ignoring their
// levels' displacements (each displaced case then read).
func TestBrepModifyRecognisesThePrism(t *testing.T) {
	t.Parallel()
	read := func(t *testing.T, bp brepPayload, k int) (brepPrismRead, bool) {
		t.Helper()
		embeds, err := brepEmbeds(bp.faces)
		require.NoError(t, err)
		r, ok, err := recognisePrism(t.Context(), bp, embeds, k)
		require.NoError(t, err)
		return r, ok
	}
	t.Run("S1", func(t *testing.T) {
		t.Parallel()
		_, s1 := internalCrossDrilled(t)
		bp := s1.payload.(brepPayload)
		for _, k := range []int{0, 2} {
			_, ok := read(t, bp, k)
			require.False(t, ok, "S1 reads as no prism along axis %d", k)
		}
		r, ok := read(t, bp, 1)
		require.True(t, ok)
		frame, _, err := brepgeom.AxisFrame(bp.faces[0].frame, 1)
		require.NoError(t, err)
		require.Equal(t, frame, r.pp.frame)
		require.Equal(t, r3.NewVec(0, 0, 1), r.pp.frame.U())
		require.Equal(t, r3.NewVec(1, 0, 0), r.pp.frame.V())
		require.Equal(t, r3.NewVec(0, 1, 0), r.pp.frame.N())
		line := func(u0, v0, u1, v1 float64) CurveSegment {
			return LineSeg{Start: Point2{U: u0, V: v0}, End: Point2{U: u1, V: v1}, TStart: 0, TEnd: 1}
		}
		want := ProfileRecord{
			Outer: LoopRecord{Segments: []CurveSegment{
				line(0, 40, 0, 0), line(0, 0, 20, 0), line(20, 0, 20, 40), line(20, 40, 0, 40),
			}},
			Holes: []LoopRecord{{Segments: []CurveSegment{
				CircleSeg{Center: Point2{U: 10, V: 20}, Radius: units.Millimeters(3), CCW: false, TStart: 1, TEnd: 0},
			}}},
		}
		require.Equal(t, want, r.pp.profile)
		require.Equal(t, [2]float64{0, 20}, [2]float64{r.pp.z0, r.pp.z1})
		require.Zero(t, r.pp.sectionDelta)
		require.Equal(t, bp.xform, r.pp.xform)
	})
	t.Run("hand-built", func(t *testing.T) {
		t.Parallel()
		_, ok := read(t, internalCrossDrilledBrep(t), 1)
		require.True(t, ok)
	})
	t.Run("wall against its material", func(t *testing.T) {
		t.Parallel()
		bp := internalCrossDrilledBrep(t)
		bp.faces[6].wall = CircleSeg{Center: Point2{U: 20, V: 10}, Radius: units.Millimeters(3), CCW: true, TStart: 0, TEnd: 1}
		_, ok := read(t, bp, 1)
		require.False(t, ok)
	})
	t.Run("rectangle facing into the material", func(t *testing.T) {
		t.Parallel()
		bp := internalCrossDrilledBrep(t)
		bp.faces[0].outward = true
		_, ok := read(t, bp, 1)
		require.False(t, ok)
	})
	// A rectangle's levels are section coordinates of the prism, so one
	// carrying a displacement reads as no prism: the hand-built z = 0 face
	// (P2(b)) and the Cut's x = 40 wall (P2(c)).
	t.Run("displaced planar rectangle", func(t *testing.T) {
		t.Parallel()
		bp := internalCrossDrilledBrep(t)
		bp.faces[0].z0Delta, bp.faces[0].z1Delta = 1e-9, 1e-9
		_, ok := read(t, bp, 1)
		require.False(t, ok)
	})
	t.Run("displaced swept rectangle", func(t *testing.T) {
		t.Parallel()
		_, s1 := internalCrossDrilled(t)
		bp := s1.payload.(brepPayload)
		faces := slices.Clone(bp.faces)
		require.False(t, faces[0].planar())
		faces[0].z1Delta = 1e-9
		bp.faces = faces
		_, ok := read(t, bp, 1)
		require.False(t, ok)
	})
}

// internalTwoHoleBrep is S1 with two Ø6 holes along y, A through (10, ·, 10)
// and B through (30, ·, 10), built by hand: the y = 20 face lists A then B,
// the y = 0 face B then A.
func internalTwoHoleBrep(t *testing.T) brepPayload {
	t.Helper()
	bp := internalCrossDrilledBrep(t)
	hole := func(x float64) LoopRecord {
		return LoopRecord{Segments: []CurveSegment{
			CircleSeg{Center: Point2{U: x, V: 10}, Radius: units.Millimeters(3), CCW: false, TStart: 1, TEnd: 0},
		}}
	}
	outer := bp.faces[4].region.Outer
	bottom := ProfileRecord{Outer: outer, Holes: []LoopRecord{hole(30), hole(10)}}
	top := ProfileRecord{Outer: outer, Holes: []LoopRecord{hole(10), hole(30)}}
	bp.faces[4].region, bp.faces[5].region = &bottom, &top
	wallB := bp.faces[6]
	bp.faces[6].wall = hole(10).Segments[0]
	wallB.wall = hole(30).Segments[0]
	bp.faces = append(bp.faces, wallB)
	bp.assignRoles()
	return bp
}

// TestBrepModifyRoutePMapsTheStartCapsHoles pins §4.1 P4's hole matching on
// the start cap: the recognised section keeps the end cap's hole order, so a
// chamfer of the start cap's hole A must reach section hole A, though the
// start cap lists it second. Shown to fail with brepPrismRead.caps leaving
// startLoop nil (the band formed around hole B at x = 30).
func TestBrepModifyRoutePMapsTheStartCapsHoles(t *testing.T) {
	t.Parallel()
	doc := New()
	body := internalCommitBrep(t, doc, internalTwoHoleBrep(t))
	rim := internalCrossDrilledBrepRim(t, body, r3.NewVec(10, 0, 10))
	result, err := body.Chamfer(t.Context(), Edges(Circular(), EndpointAt(rim.start.position)).Exactly(1), units.Millimeters(1))
	require.NoError(t, err)
	lo, hi := piEnclosed(big.NewRat(16000, 1), big.NewRat(-1090, 3))
	requireCoversInterval(t, result.volume, lo, hi)
	cones := 0
	for _, f := range result.Faces() {
		cone, ok := f.Surface().(Cone)
		if !ok {
			continue
		}
		cones++
		// Hole A's axis is at x = 10 and hole B's at x = 30.
		require.Less(t, math.Abs(cone.Origin.X-10), 1.0, "the band is around hole A, at %v", cone.Origin)
	}
	require.Equal(t, 1, cones)
}

// TestBrepModifySplitsThePrismReading pins the two halves recognisePrism is
// built from (docs/modify-general-design.md PR 0): readPrismCaps reads P1, P3
// and P4 alone, and classifyPrismWalls P2 and P5. On the hand-built cross-
// drilled box the caps along y are the two xz faces with the hole as their
// one inner loop, and the walls classify; along z the caps read although the
// hole's wall is no wall of that prism, which only the second half sees.
func TestBrepModifySplitsThePrismReading(t *testing.T) {
	t.Parallel()
	bp := internalCrossDrilledBrep(t)
	embeds, err := brepEmbeds(bp.faces)
	require.NoError(t, err)

	caps, ok := readPrismCaps(bp, embeds, 1)
	require.True(t, ok)
	require.Equal(t, [2]int{4, 5}, [2]int{caps.bottom, caps.top})
	require.Equal(t, [2]float64{0, 20}, [2]float64{caps.zlo, caps.zhi})
	require.Len(t, caps.section.Outer.Segments, 4)
	require.Len(t, caps.section.Holes, 1)
	require.Equal(t, []int{0, 1}, caps.bottomLoop)
	walls, ok, err := classifyPrismWalls(t.Context(), bp, embeds, 1, caps)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, walls.walls, 1, `the hole's wall`)
	require.Len(t, walls.rects, 4)

	capsZ, ok := readPrismCaps(bp, embeds, 2)
	require.True(t, ok)
	require.Equal(t, [2]float64{0, 20}, [2]float64{capsZ.zlo, capsZ.zhi})
	require.Empty(t, capsZ.section.Holes)
	_, ok, err = classifyPrismWalls(t.Context(), bp, embeds, 2, capsZ)
	require.NoError(t, err)
	require.False(t, ok)
}
