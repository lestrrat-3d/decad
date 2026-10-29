package export

import (
	"math"
	"slices"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures write agent-docs/loop-fillet-design.md's §8 bodies through the
// analytic arm: P3's root and rim, P8's top loop and P1's top loop. Body
// .Tessellate refuses a fillet-banded body until the design's PR F-2, and
// NewSTEPFile tessellates first as the shared admission check, so the tests
// run supportsAnalyticSTEP and assembleSTEPFile, the two steps NewSTEPFile
// takes after that check, on the real body.

var bandUp = r3.NewVec(0, 0, 1)

func bandHeader() step.Header {
	return step.Header{
		Description:   []string{"banded AP214 test"},
		Name:          "band.step",
		Timestamp:     time.Date(2026, 10, 9, 0, 0, 0, 0, time.UTC),
		Authors:       []string{"Decad"},
		Organizations: []string{"Decad"},
	}
}

func bandBox(t *testing.T, doc *decad.Document, x0, y0, x1, y1, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// bandDrilledBox is the 40x20x20 box cut by a O6 hole along y through
// (20, ., 10), after filleting its four vertical edges by round when it is
// positive: P1 without round, P8 with round = 3.
func bandDrilledBox(t *testing.T, round float64) *decad.Body {
	t.Helper()
	doc := decad.New()
	body := bandBox(t, doc, 0, 0, 40, 20, 20)
	if round > 0 {
		var err error
		body, err = decad.Fillet(t.Context(), body, decad.Edges(decad.ParallelTo(bandUp)).Exactly(4), units.Millimeters(round))
		require.NoError(t, err)
	}
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XZ(), -10)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(20, 10)
	s.Fix(c)
	s.CreateCircle(c, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	drill, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(11)})
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), body, drill)
	require.NoError(t, err)
	return out
}

// bandRoundBoss is P3: the 40x40x10 plate unioned with a O10 boss about
// (20, 20) standing from z = 10 to z = 25.
func bandRoundBoss(t *testing.T) *decad.Body {
	t.Helper()
	doc := decad.New()
	plate := bandBox(t, doc, 0, 0, 40, 40, 10)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(20, 20)
	s.Fix(c)
	s.CreateCircle(c, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	boss, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(15), Dir: decad.Along})
	require.NoError(t, err)
	out, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	return out
}

// bandFacing is the one planar face facing n through z = level.
func bandFacing(t *testing.T, body *decad.Body, n r3.Vec, level float64) *decad.Face {
	t.Helper()
	var found []*decad.Face
	for _, f := range body.Faces() {
		pl, ok := f.Surface().(decad.Plane)
		if !ok || math.Abs(pl.Frame.Origin().Z-level) > 1e-9 {
			continue
		}
		normal, err := f.NormalAt(f.Loops()[0].CoEdges()[0].Start().Position().Value)
		require.NoError(t, err)
		if normal.Value.Sub(n).Len() < 1e-12 {
			found = append(found, f)
		}
	}
	require.Len(t, found, 1)
	return found[0]
}

// bandLoopQuery selects exactly the edges of loop.
func bandLoopQuery(loop *decad.Loop) *decad.EdgeQuery {
	var q *decad.EdgeQuery
	edges := loop.Edges()
	for _, e := range edges {
		a, b := e.Start().Position().Value, e.End().Position().Value
		preds := []decad.EdgePredicate{decad.EndpointAt(a), decad.EndpointAt(b)}
		if _, line := e.Curve().(decad.Line3); line {
			preds = append(preds, decad.ParallelTo(b.Sub(a)))
		} else {
			preds = append(preds, decad.Circular())
		}
		if q == nil {
			q = decad.Edges(preds...)
			continue
		}
		q.Or(preds...)
	}
	return q.Exactly(len(edges))
}

// bandFillet fillets loop li of the planar face facing +z at level by r.
func bandFillet(t *testing.T, body *decad.Body, level float64, li int, r float64) *decad.Body {
	t.Helper()
	face := bandFacing(t, body, bandUp, level)
	out, err := decad.Fillet(t.Context(), body, bandLoopQuery(face.Loops()[li]), units.Millimeters(r))
	require.NoError(t, err)
	return out
}

// bandFile writes body through the analytic arm and requires that arm to
// take it.
func bandFile(t *testing.T, body *decad.Body) step.File {
	t.Helper()
	ok, err := supportsAnalyticSTEP(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok, "the analytic arm takes the banded body")
	return armFile(t, body)
}

// armFile writes body through the analytic arm's writer alone, whatever
// supportsAnalyticSTEP says of it. NewSTEPFile never does this for a body the
// admission refuses; the tests that call it directly state what the
// admission says and why the writer is still read on that body.
func armFile(t *testing.T, body *decad.Body) step.File {
	t.Helper()
	f, err := assembleSTEPFile(t.Context(), body, bandHeader(), true, nil, nil, nil, nil, nil)
	require.NoError(t, err)
	data, err := f.Marshal()
	require.NoError(t, err)
	require.Contains(t, string(data), "analytic decad solid")
	return f
}

// bandEntities groups a file's entities by name.
func bandEntities(f step.File) map[string][]step.Entity {
	out := map[string][]step.Entity{}
	for _, e := range f.Entities {
		out[e.Name] = append(out[e.Name], e)
	}
	return out
}

// bandReals reads the real attributes at index i of every entity, sorted.
func bandReals(entities []step.Entity, i int) []float64 {
	var out []float64
	for _, e := range entities {
		out = append(out, float64(e.Parameters[i].(step.Real)))
	}
	slices.Sort(out)
	return out
}

// requireEveryEdgeCurveInBothSenses requires each EDGE_CURVE to be used by one
// ORIENTED_EDGE in each sense, which a closed shell whose faces' loops each
// run about their own face normal requires.
func requireEveryEdgeCurveInBothSenses(t *testing.T, f step.File) {
	t.Helper()
	uses := map[step.Reference][]step.Enumeration{}
	for _, e := range f.Entities {
		if e.Name == "ORIENTED_EDGE" {
			edge := e.Parameters[3].(step.Reference)
			uses[edge] = append(uses[edge], e.Parameters[4].(step.Enumeration))
		}
	}
	require.NotEmpty(t, uses)
	for edge, senses := range uses {
		require.ElementsMatch(t, []step.Enumeration{"T", "F"}, senses, "EDGE_CURVE #%d", edge)
	}
}

// faceLoopPoints reads the vertices an ADVANCED_FACE's outer bound walks, in
// walking order, from the file alone: each ORIENTED_EDGE contributes its
// start vertex, the EDGE_CURVE's start when the sense is T and its end when F.
func faceLoopPoints(t *testing.T, f step.File, face step.Entity) []r3.Vec {
	t.Helper()
	byID := map[uint64]step.Entity{}
	for _, e := range f.Entities {
		byID[e.ID] = e
	}
	ref := func(v step.Value) step.Entity { return byID[uint64(v.(step.Reference))] }
	point := func(vertex step.Entity) r3.Vec {
		p := ref(vertex.Parameters[1])
		list := p.Parameters[1].(step.List)
		return r3.NewVec(float64(list[0].(step.Real)), float64(list[1].(step.Real)), float64(list[2].(step.Real)))
	}
	var out []r3.Vec
	for _, b := range face.Parameters[1].(step.List) {
		bound := ref(b)
		loop := ref(bound.Parameters[1])
		for _, oe := range loop.Parameters[1].(step.List) {
			oriented := ref(oe)
			edge := ref(oriented.Parameters[3])
			from := edge.Parameters[1]
			if oriented.Parameters[4].(step.Enumeration) == "F" {
				from = edge.Parameters[2]
			}
			out = append(out, point(ref(from)))
		}
	}
	return out
}

// requireFacesRunCounterClockwiseAboutOutwardNormal reads every ADVANCED_FACE
// whose body face satisfies pick back from the file and requires its outer
// loop's chord polygon to have a vector area along the body face's outward
// normal at the polygon's own vertices: the loop runs counter-clockwise about
// the face normal. Faces are written in Body.Faces order. The oracle never
// reads the production orientation code; it reads only the file and decad's
// own face normal.
func requireFacesRunCounterClockwiseAboutOutwardNormal(
	t *testing.T, f step.File, body *decad.Body, pick func(*decad.Face) bool,
) int {
	t.Helper()
	checked := 0
	faces := bandEntities(f)["ADVANCED_FACE"]
	require.Len(t, faces, len(body.Faces()))
	for i, face := range faces {
		if !pick(body.Faces()[i]) {
			continue
		}
		pts := faceLoopPoints(t, f, face)
		area, mean := r3.Vec{}, r3.Vec{}
		for j, p := range pts {
			area = area.Add(p.Cross(pts[(j+1)%len(pts)]).Scale(0.5))
			n, err := body.Faces()[i].NormalAt(p)
			require.NoError(t, err)
			mean = mean.Add(n.Value)
		}
		require.Positive(t, area.Dot(mean), "face %d loop runs clockwise about its outward normal", i)
		checked++
	}
	return checked
}

// TestAnalyticBandRoundBoss checks analytic STEP output for P3's root and rim fillets.
// A boss root fill
// is a concave whole-turn torus (major 6, minor 1; its sense is F, since the
// body's outward normal points toward the tube's centre circle); the rim is a
// convex one (major 4, minor 1; sense T). Each file holds the plate's six
// planes plus the boss top, the boss wall as one full cylinder, one torus
// face, and four CIRCLE entities: the three boundary circles and the torus's
// synthetic seam, a tube meridian of radius 1. Shown to fail with
// supportsAnalyticSTEP's Torus arm deleted (the body fell to the faceted
// writer) and with torusFaceSense inverted (root and rim swapped senses).
func TestAnalyticBandRoundBoss(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name           string
		level          float64
		li             int
		major, minor   float64
		sense          step.Enumeration
		circleRadii    []float64
		faces, planes  int
		cylinderRadius float64
	}{
		{"root", 10, 1, 6, 1, "F", []float64{1, 5, 5, 6}, 9, 7, 5},
		{"rim", 25, 0, 4, 1, "T", []float64{1, 4, 5, 5}, 9, 7, 5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			body := bandFillet(t, bandRoundBoss(t), tc.level, tc.li, 1)
			f := bandFile(t, body)
			byName := bandEntities(f)
			require.Len(t, byName["ADVANCED_FACE"], tc.faces)
			require.Len(t, byName["PLANE"], tc.planes)
			require.Equal(t, []float64{tc.cylinderRadius}, bandReals(byName["CYLINDRICAL_SURFACE"], 2))
			require.Len(t, byName["TOROIDAL_SURFACE"], 1)
			require.Equal(t, []float64{tc.major}, bandReals(byName["TOROIDAL_SURFACE"], 2))
			require.Equal(t, []float64{tc.minor}, bandReals(byName["TOROIDAL_SURFACE"], 3))
			require.Equal(t, tc.circleRadii, bandReals(byName["CIRCLE"], 2))
			require.Empty(t, byName["ELLIPSE"])
			requireEveryEdgeCurveInBothSenses(t, f)
			torusRef := byName["TOROIDAL_SURFACE"][0].ID
			torusFaces := 0
			for _, face := range byName["ADVANCED_FACE"] {
				if uint64(face.Parameters[2].(step.Reference)) == torusRef {
					require.Equal(t, tc.sense, face.Parameters[3])
					torusFaces++
				}
			}
			require.Equal(t, 1, torusFaces)
		})
	}
}

func isTorus(f *decad.Face) bool {
	_, ok := f.Surface().(decad.Torus)
	return ok
}

func isCylinderPatch(f *decad.Face) bool {
	_, ok := f.Surface().(decad.Cylinder)
	return ok && len(f.Loops()) == 1
}

// TestAnalyticBandRoundedPlateTopLoop writes P8's top loop, r = 2 on the box
// whose four vertical edges are filleted r = 3: four spindle torus patches
// (major R - r = 1, minor 2, each bounded by two parallel arcs and two
// quarter meridians) between four straight-walk cylinder patches, whose ends
// are those meridians. Besides the box's six planes there are nine
// cylinders (four corner walls, the bore, four patches) and no ellipse. The
// circle radii are the closed forms: the four top contours at 1, the eight
// meridians at 2, and ten arcs and circles at 3 (the corner walls' two rims
// each and the bore's two circles). Every torus patch's loop runs
// counter-clockwise about its outward normal, read back from the file, and
// every arc holds both its vertices in its own plane: the fillet build holds
// each tangent join's foot at the exact point the record denotes, so the
// meridian at the (0, 0) corner stands at y = 3 with both its vertices
// (offset2d.ExactCornerFeet). Shown to fail with supportsAnalyticTorusPatch
// refusing every loop (the body fell to the faceted writer), with
// torusPatchArea's parallel sweep sign inverted (the patches then walked
// clockwise), with torusFaceSense inverted, and with ExactCornerFeet's G1
// arm deleted (on amd64 that meridian's Center was held at y = 2.9999999999999996 off its
// end vertex's plane, and the admission refused the body).
func TestAnalyticBandRoundedPlateTopLoop(t *testing.T) {
	t.Parallel()
	body := bandFillet(t, bandDrilledBox(t, 3), 20, 0, 2)
	arcs := 0
	for _, face := range body.Faces() {
		if _, ok := face.Surface().(decad.Torus); !ok {
			continue
		}
		for _, ce := range face.Loops()[0].CoEdges() {
			arc := ce.Edge().Curve().(decad.Arc3)
			require.True(t, arcInItsPlane(ce.Edge(), arc), "the arc at %v about %v holds its vertices", arc.Center, arc.Axis)
			arcs++
		}
	}
	require.Equal(t, 16, arcs, "four torus patches of four arcs")
	f := bandFile(t, body)
	byName := bandEntities(f)
	require.Len(t, byName["ADVANCED_FACE"], 19)
	require.Len(t, byName["PLANE"], 6)
	require.Len(t, byName["CYLINDRICAL_SURFACE"], 9)
	require.Equal(t, []float64{1, 1, 1, 1}, bandReals(byName["TOROIDAL_SURFACE"], 2))
	require.Equal(t, []float64{2, 2, 2, 2}, bandReals(byName["TOROIDAL_SURFACE"], 3))
	require.Equal(t, append(append(slices.Repeat([]float64{1}, 4), slices.Repeat([]float64{2}, 8)...),
		slices.Repeat([]float64{3}, 10)...), bandReals(byName["CIRCLE"], 2))
	require.Empty(t, byName["ELLIPSE"])
	requireEveryEdgeCurveInBothSenses(t, f)
	require.Equal(t, 4, requireFacesRunCounterClockwiseAboutOutwardNormal(t, f, body, isTorus))
}

// TestAnalyticBandCrossDrilledTopLoop writes P1's top loop, r = 2: four
// quarter-cylinder patches meeting along four ELLIPSE edges of semi-axes
// 2 sqrt 2 and 2, each patch a loop of two lines and two ellipses. The box's
// six planes, the bore and the four patches are eleven faces, and the only
// circles are the bore's two (radius 3). Every patch's loop runs
// counter-clockwise about its outward normal, read back from the file. Shown
// to fail with supportsAnalyticPartialWall's Ellipse3 case deleted (the body
// fell to the faceted writer), with addEdge's Ellipse3 case deleted (the
// build refused), and with partialWallArea's ellipse advance deleted (the
// patches then walked clockwise or the build refused as unoriented).
func TestAnalyticBandCrossDrilledTopLoop(t *testing.T) {
	t.Parallel()
	body := bandFillet(t, bandDrilledBox(t, 0), 20, 0, 2)
	f := bandFile(t, body)
	byName := bandEntities(f)
	require.Len(t, byName["ADVANCED_FACE"], 11)
	require.Len(t, byName["PLANE"], 6)
	require.Len(t, byName["CYLINDRICAL_SURFACE"], 5)
	require.Empty(t, byName["TOROIDAL_SURFACE"])
	require.Equal(t, []float64{3, 3}, bandReals(byName["CIRCLE"], 2))
	require.Len(t, byName["ELLIPSE"], 4)
	for _, semi := range bandReals(byName["ELLIPSE"], 2) {
		require.InDelta(t, 2*math.Sqrt2, semi, 1e-12)
	}
	require.Equal(t, []float64{2, 2, 2, 2}, bandReals(byName["ELLIPSE"], 3))
	requireEveryEdgeCurveInBothSenses(t, f)
	require.Equal(t, 4, requireFacesRunCounterClockwiseAboutOutwardNormal(t, f, body, isCylinderPatch))
}

// isHornTorusPatch picks a torus face of one three-edge loop.
func isHornTorusPatch(f *decad.Face) bool {
	return isTorus(f) && len(f.Loops()) == 1 && isHornPatch(f.Loops()[0])
}

// requireFileVerticesOnTori reads every TOROIDAL_SURFACE ADVANCED_FACE back
// from the file and requires each vertex its loop walks to lie on the torus
// the file states: (ρ − R)² + h² = r², h the vertex's height along the
// placement axis from its location and ρ its distance from that axis. It
// returns the stated location of each face it checked. The oracle reads only
// the file; 1e-9 mm² is far above the rounding of these coordinates and far
// below any wrong placement or radius.
func requireFileVerticesOnTori(t *testing.T, f step.File) []r3.Vec {
	t.Helper()
	byID := map[uint64]step.Entity{}
	for _, e := range f.Entities {
		byID[e.ID] = e
	}
	ref := func(v step.Value) step.Entity { return byID[uint64(v.(step.Reference))] }
	triple := func(e step.Entity) r3.Vec {
		list := e.Parameters[1].(step.List)
		return r3.NewVec(float64(list[0].(step.Real)), float64(list[1].(step.Real)), float64(list[2].(step.Real)))
	}
	var locations []r3.Vec
	for _, face := range bandEntities(f)["ADVANCED_FACE"] {
		surface := ref(face.Parameters[2])
		if surface.Name != "TOROIDAL_SURFACE" {
			continue
		}
		placement := ref(surface.Parameters[1])
		location, axis := triple(ref(placement.Parameters[1])), triple(ref(placement.Parameters[2]))
		major, minor := float64(surface.Parameters[2].(step.Real)), float64(surface.Parameters[3].(step.Real))
		for _, p := range faceLoopPoints(t, f, face) {
			d := p.Sub(location)
			h := d.Dot(axis)
			rho := d.Sub(axis.Scale(h)).Len()
			require.InDelta(t, minor*minor, (rho-major)*(rho-major)+h*h, 1e-9, "vertex %v off the torus", p)
		}
		locations = append(locations, location)
	}
	return locations
}

// TestAnalyticBandPocketMouthHornTori writes P2's pocket mouth fillet, r =
// 1.5, analytic. Each of the mouth's four reflex corners is a horn torus patch
// (loop fillet LF6): a TOROIDAL_SURFACE with major and minor radius 1.5,
// located at the pocket corner 1.5 below the top, (10, 15), (10, 25), (30, 25)
// and (30, 15) at z = 8.5, bounded by its connector arc and the two meridians
// that meet at that apex. The plate's six planes, the pocket's floor and four
// walls, the four straight-walk cylinder patches of radius 1.5 and the four
// horn patches are nineteen faces. The circles are the four connector arcs and
// eight meridians, each of radius 1.5, and there is no ellipse. Every vertex
// lies on the torus the file states, every edge is used once in each
// direction, each horn patch's loop runs counter-clockwise about its outward
// normal, read back from the file, and each is of sense T: the fillet rounds
// a convex edge, so the outward normal points away from the tube's centre
// circle as the torus's own does. Shown to fail with supportsAnalyticTorus
// sending a three-edge loop to supportsAnalyticTorusPatch (the body fell to
// the faceted writer), with isWalkedPatch refusing a torus (the build refused
// a patch with no decided walk), with patchWalks' decided walk inverted (the
// horn patches' edges were used twice in one direction), with torusFaceSense
// inverted (the senses read F), and with addTorusFace placing the torus 0.5
// along its axis (the vertices left the stated torus). torusOffAxisVertex's
// skip of an apex vertex is not shown red: no fixture's horn loop starts at
// its apex, so the first vertex already lies off the axis.
func TestAnalyticBandPocketMouthHornTori(t *testing.T) {
	t.Parallel()
	body := bandPocketMouthFillet(t)
	f := bandFile(t, body)
	byName := bandEntities(f)
	require.Len(t, byName["ADVANCED_FACE"], 19)
	require.Len(t, byName["PLANE"], 11)
	require.Equal(t, slices.Repeat([]float64{1.5}, 4), bandReals(byName["CYLINDRICAL_SURFACE"], 2))
	require.Equal(t, slices.Repeat([]float64{1.5}, 4), bandReals(byName["TOROIDAL_SURFACE"], 2))
	require.Equal(t, slices.Repeat([]float64{1.5}, 4), bandReals(byName["TOROIDAL_SURFACE"], 3))
	require.Equal(t, slices.Repeat([]float64{1.5}, 12), bandReals(byName["CIRCLE"], 2))
	require.Empty(t, byName["ELLIPSE"])
	require.ElementsMatch(t, []r3.Vec{
		r3.NewVec(10, 15, 8.5), r3.NewVec(10, 25, 8.5), r3.NewVec(30, 25, 8.5), r3.NewVec(30, 15, 8.5),
	}, requireFileVerticesOnTori(t, f))
	require.Equal(t, slices.Repeat([]step.Enumeration{"T"}, 4), patchSenses(f, "TOROIDAL_SURFACE"))
	requireEveryEdgeCurveInBothSenses(t, f)
	require.Equal(t, 4, requirePatchFacesRunCounterClockwise(t, f, body, isHornTorusPatch))
}

// TestHornPatchAdmissionRefusesForeignTags runs supportsAnalyticHornPatch on a
// real horn patch loop of the pocket mouth fillet against tags that do not
// hold it, each failing one leg: a torus whose Minor is 1.6, not its Major
// (no longer a horn torus), one whose Center moved 1 along its axis (no loop
// vertex is its apex), and one about the x axis, under which the connector
// arc reads as a meridian and the meridian about x as a parallel, so an edge
// at the apex is not a meridian, and a horn torus of radii 1.6 and 1.6 at the
// apex, whose tube the loop's meridians of radius 1.5 do not run round. Each
// was shown to fail with its leg deleted (the 1.6 horn with torusPatchArc's
// meridian radius leg deleted). The one-parallel leg is not shown red: a real three-edge loop's arcs lie
// about three mutually perpendicular axes, so any torus axis under which all
// three classify reads exactly one of them as a parallel.
func TestHornPatchAdmissionRefusesForeignTags(t *testing.T) {
	t.Parallel()
	var horn *decad.Face
	for _, face := range bandPocketMouthFillet(t).Faces() {
		if isHornTorusPatch(face) {
			horn = face
			break
		}
	}
	require.NotNil(t, horn)
	tag := horn.Surface().(decad.Torus)
	loop := horn.Loops()[0]
	require.True(t, supportsAnalyticHornPatch(loop, tag))
	spindle := tag
	spindle.Minor = units.Millimeters(1.6)
	require.False(t, supportsAnalyticHornPatch(loop, spindle), "a horn patch needs Minor equal to Major")
	moved := tag
	moved.Center = tag.Center.Add(tag.Axis)
	require.False(t, supportsAnalyticHornPatch(loop, moved), "a horn patch needs a vertex at its Center")
	turned := tag
	turned.Axis = r3.NewVec(1, 0, 0)
	require.False(t, supportsAnalyticHornPatch(loop, turned), "both edges at the apex must be meridians")
	wider := tag
	wider.Major, wider.Minor = units.Millimeters(1.6), units.Millimeters(1.6)
	require.False(t, supportsAnalyticHornPatch(loop, wider), "each meridian's radius must be the tag's Minor")
}

// TestTorusPatchAdmissionRefusesAForeignMinor runs supportsAnalyticTorusPatch
// on the real four-arc patch loops of P8's top-loop fillet, r = 2 (spindle
// torus patches of major 1 and minor 2), and against each tag with its Minor
// set to 2.5, which the loop's meridians of radius 2 do not run round. Shown
// to fail with torusPatchArc's meridian radius leg deleted (the foreign tag
// was admitted).
func TestTorusPatchAdmissionRefusesAForeignMinor(t *testing.T) {
	t.Parallel()
	checked := 0
	for _, face := range bandFillet(t, bandDrilledBox(t, 3), 20, 0, 2).Faces() {
		tag, ok := face.Surface().(decad.Torus)
		if !ok {
			continue
		}
		require.True(t, supportsAnalyticTorusPatch(face.Loops()[0], tag))
		wider := tag
		wider.Minor = units.Millimeters(2.5)
		require.False(t, supportsAnalyticTorusPatch(face.Loops()[0], wider), "each meridian's radius must be the tag's Minor")
		checked++
	}
	require.Equal(t, 4, checked, "four torus patches")
}

// TestAnalyticHornPatchWithBoundedVertexKeepsFaceted builds the pocket mouth
// fillet with the plate and pocket moved by (0.1, 0.1), whose horn patch
// vertices carry rounding bounds, and requires the face admission to refuse
// it. Shown to fail with supportsAnalyticHornPatch's loopVerticesExact leg
// deleted (the face admission took the body).
func TestAnalyticHornPatchWithBoundedVertexKeepsFaceted(t *testing.T) {
	t.Parallel()
	body := bandPocketMouthFilletAt(t, 0.1)
	require.True(t, tagsUndeparted(body))
	bounded := 0
	for _, face := range body.Faces() {
		if isHornTorusPatch(face) && !loopVerticesExact(face.Loops()[0]) {
			bounded++
		}
	}
	require.Positive(t, bounded, "a horn patch has a vertex with a bound")
	ok, err := supportsAnalyticFaces(t.Context(), body)
	require.NoError(t, err)
	require.False(t, ok, "a horn patch vertex with a bound sends the body to the faceted writer")
}

// TestTorusPatchArcRefusesAMeridianOffItsPlane runs torusPatchArc on the real
// meridians of the pocket mouth fillet's horn torus patches (loop-fillet
// LF6), each a quarter circle about the walk direction of the wall it
// bounds, and on the same edges with the Axis swapped for the other walk's
// direction: the torus axis crossed with the meridian's own, still
// perpendicular to the torus axis, so only arcInItsPlane can refuse it. The
// swapped Axis is the one an LF6 trailing meridian carried before it took its
// own wall's axis, and its circle's plane misses the meridian's vertices.
// torusPatchArc is the per-arc admission both a horn patch and a four-edge
// patch pass through. Shown to fail with torusPatchArc's arcInItsPlane leg
// deleted (the swapped meridian was admitted).
func TestTorusPatchArcRefusesAMeridianOffItsPlane(t *testing.T) {
	t.Parallel()
	checked := 0
	for _, face := range bandPocketMouthFillet(t).Faces() {
		torus, ok := face.Surface().(decad.Torus)
		if !ok || len(face.Loops()) != 1 {
			continue
		}
		require.Len(t, face.Loops()[0].CoEdges(), 3, "a horn torus patch")
		require.True(t, supportsAnalyticTorus(face.Loops(), torus))
		for _, ce := range face.Loops()[0].CoEdges() {
			arc, ok := ce.Edge().Curve().(decad.Arc3)
			if !ok || !torusArcIsMeridian(arc, torus) {
				continue
			}
			parallel, ok := torusPatchArc(ce.Edge(), arc, torus)
			require.True(t, ok, "the meridian at %v about %v holds its own vertices", arc.Center, arc.Axis)
			require.False(t, parallel)
			swapped := arc
			swapped.Axis = torus.Axis.Cross(arc.Axis)
			require.True(t, torusArcIsMeridian(swapped, torus))
			require.False(t, arcInItsPlane(ce.Edge(), swapped))
			_, ok = torusPatchArc(ce.Edge(), swapped, torus)
			require.False(t, ok, "the meridian at %v about the swapped %v is off its circle's plane", arc.Center, swapped.Axis)
			checked++
		}
	}
	require.Equal(t, 8, checked, "two meridians at each of the mouth's four reflex corners")
}

// bandPocketMouthFillet is P2: the 40x40x10 plate with a 20x10 pocket 5 deep,
// its mouth filleted r = 1.5. Each of the mouth's four reflex corners is a
// horn torus patch bounded by a connector arc and two meridians.
func bandPocketMouthFillet(t *testing.T) *decad.Body {
	t.Helper()
	return bandPocketMouthFilletAt(t, 0)
}

// bandPocketMouthFilletAt is bandPocketMouthFillet with the plate and the
// pocket moved by (off, off).
func bandPocketMouthFilletAt(t *testing.T, off float64) *decad.Body {
	t.Helper()
	doc := decad.New()
	plate := bandBox(t, doc, off, off, 40+off, 40+off, 10)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 5)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	r := s.CreateRectangle(10+off, 15+off, 30+off, 25+off)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	return bandFillet(t, cut, 10, 1, 1.5)
}
