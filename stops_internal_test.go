package decad

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// stubExtent is a payload whose directional extent publishes exactly the far
// side and displacement a case asks for. The in-path test's three outcomes turn
// on where that displacement sits against the sketch plane, and driving them
// through real geometry would mean hunting for a section whose bracket lands on
// the scale-relative contact tolerance itself.
type stubExtent struct{ hi, bound float64 }

func (stubExtent) Transform() r3.Transform { return r3.Identity() }

func (stubExtent) Placed(context.Context, topo.Env, r3.Transform) (*topo.Body, error) {
	return nil, ErrUnsupported
}

func (s stubExtent) ExtentAlong(r3.Vec) (float64, float64, float64, error) {
	return -s.hi, s.hi, s.bound, nil
}

// TestResolveThroughAllDecidesInPathOutsideDisplacement covers the three
// outcomes of the in-path test (docs/spline-design.md §6.4): a far side beyond
// the sketch plane by more than its own displacement is met and charges that
// displacement to the level, one short of the plane by more than it is not met,
// and one the displacement straddles the plane with refuses rather than guess a
// dependency.
func TestResolveThroughAllDecidesInPathOutsideDisplacement(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	doc := func(hi, bound float64) *Document {
		return bornDocument(&Document{n: topo.Document{
			LiveSet: []*topo.Body{bodyNode(bornBody(&Body{
				n: topo.Body{Origin: provenance.Ref{Producer: 1}, Payload: stubExtent{hi: hi, bound: bound}},
			}))},
		}})
	}

	t.Run("beyond the plane", func(t *testing.T) {
		stop, delta, refs, err := doc(10, 1).resolveThroughAll(frame, 1)
		require.NoError(t, err)
		require.Equal(t, 10.0, stop, "the sweep stops at the held far side")
		require.Equal(t, []producerID{1}, refs)
		require.GreaterOrEqual(t, delta, 1.0, "the level carries the extent's own displacement")
	})

	t.Run("short of the plane", func(t *testing.T) {
		_, _, _, err := doc(-10, 1).resolveThroughAll(frame, 1)
		require.ErrorIs(t, err, ErrDegenerate, "no body is in the sweep's path")
	})

	t.Run("straddling the plane", func(t *testing.T) {
		_, _, _, err := doc(0.5, 1).resolveThroughAll(frame, 1)
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "cannot decide whether a body is in its path")
	})
}

// TestResolveThroughAllComposesEveryFarEndInterval keeps the recorded maximum
// unchanged while bounding a lower held endpoint that can be farther in the
// denoted geometry. It also refuses a competing interval with no finite upper
// endpoint instead of publishing an unrepresentable stop bound.
func TestResolveThroughAllComposesEveryFarEndInterval(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	profile := profileRecord{Outer: synthRectLoop(0, 0, 1, 1)}
	stopBody := func(ref producerID, z1, z1Delta float64) *Body {
		return bornBody(&Body{
			n: topo.Body{Origin: provenance.Ref{Producer: ref}, Payload: prismPayload{Payload: prismrecord.Payload{
				Profile: profile,
				Frame:   frame,
				Z1:      z1,
				Z1Delta: z1Delta,
				Xform:   r3.Identity(),
			}},
			},
		})
	}

	t.Run("bounds a lower held contender", func(t *testing.T) {
		doc := bornDocument(&Document{n: topo.Document{
			LiveSet: []*topo.Body{
				bodyNode(stopBody(1, 9.25, 1)),
				bodyNode(stopBody(2, 10, 0)),
			},
		}})

		stop, bound, refs, err := doc.resolveThroughAll(frame, 1)
		require.NoError(t, err)
		require.Equal(t, 10.0, stop, "the held farthest endpoint remains the stop")
		require.Equal(t, []producerID{1, 2}, refs, "the dependency order remains nearest-first")
		require.GreaterOrEqual(t, bound, 0.25, "the lower held endpoint can be 0.25 mm farther")
	})

	t.Run("refuses an unrepresentable contender", func(t *testing.T) {
		doc := bornDocument(&Document{n: topo.Document{
			LiveSet: []*topo.Body{
				bodyNode(stopBody(1, math.MaxFloat64, math.MaxFloat64)),
				bodyNode(stopBody(2, math.MaxFloat64, 0)),
			},
		}})

		_, _, _, err := doc.resolveThroughAll(frame, 1)
		require.ErrorIs(t, err, ErrUnsupported)
	})
}

// TestRejectAxisCrossingRefusesUnprobableCurve checks that the angular stop
// audit keeps its Line3 shortcut — a straight edge
// between two agreeing vertices never leaves the half-plane they agree on —
// but any OTHER curve kind now refuses rather than reading its own silence as
// "no crossing." A NURBSCurve rim (a free-form prism's own side-face
// boundary, once one is buildable) and a FacetedCurve rim (a boolean-built
// edge) both have no closed-form deepest-point test the way Circle3/Arc3 do,
// so admitting them through the same default arm as Line3 would bless a claim
// this audit never examined — the falsify-never-bless rule (repo CLAUDE.md).
func TestRejectAxisCrossingRefusesUnprobableCurve(t *testing.T) {
	t.Parallel()
	st := angularStops{
		a3: r3.NewVec(0, 0, 0),
		w:  r3.NewVec(0, 0, 1),
		r0: r3.NewVec(1, 0, 0),
		e1: r3.NewVec(0, 1, 0),
	}
	m := st.r0

	t.Run("Line3 stays permitted", func(t *testing.T) {
		require.NoError(t, extent.RejectAxisCrossing(st.stopGeometry(), angularStopEdge(bornEdge(&Edge{n: topo.Edge{Curve: surfacegeom.Line3{}}})), m))
	})
	for _, tc := range []struct {
		name  string
		curve surfacegeom.Curve
	}{
		{name: "NURBSCurve", curve: surfacegeom.NURBSCurve{}},
		{name: "FacetedCurve", curve: surfacegeom.FacetedCurve{}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			err := extent.RejectAxisCrossing(st.stopGeometry(), angularStopEdge(bornEdge(&Edge{n: topo.Edge{Curve: tc.curve}})), m)
			require.ErrorIs(t, err, ErrUnsupported)
			require.ErrorContains(t, err, tc.name, "the refusal names the curve type it cannot probe")
		})
	}
}

// TestFaceHalfPlaneRefusesFreeformRim proves the refusal reaches a stop face
// used as a whole, not merely the isolated helper: a planar face whose loop
// carries a NURBSCurve edge — exactly the rim a free-form prism's planar cap
// would carry once one is buildable (docs/spline-design.md §10 row P4b) —
// refuses through faceHalfPlane, the function resolveToFaceAngular calls
// right after selecting the stop face. No evaluator path builds such a face
// publicly yet (the free-form side-face build is a later increment), so the
// face is built directly, the pattern prism_boolean_internal_test.go already
// uses for a shape nothing else can build.
func TestFaceHalfPlaneRefusesFreeformRim(t *testing.T) {
	t.Parallel()
	// This frame's plane is the xz-plane, which contains the revolve axis, so
	// the fixture is a valid radial stop face and both endpoints lie on it.
	// r3.NewFrame(origin, u, v) takes u as the FIRST axis and derives
	// N = U×V, so U=(0,0,1), V=(1,0,0) gives N=(0,1,0) — not (0,0,1). Both
	// gates resolveToFaceAngular applies before faceHalfPlane pass here:
	// |N·w| = 0 and the plane's offset from a3 is 0. Loop closure is not part
	// of what this test drives: faceHalfPlane walks face.Edges(), a flat
	// concat over loops, and neither it nor boundaryProbes nor
	// extent.RejectAxisCrossing reads a loop's closure.
	planeFrame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	st := angularStops{
		a3: r3.NewVec(0, 0, 0),
		w:  r3.NewVec(0, 0, 1),
		r0: r3.NewVec(1, 0, 0),
		e1: r3.NewVec(0, 1, 0),
	}

	t.Run("NURBSCurve rim", func(t *testing.T) {
		edge := bornEdge(&Edge{
			n: topo.Edge{Curve: surfacegeom.NURBSCurve{}, Start: vertexNode(bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 0, 0)}})),
				End: vertexNode(bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 0, 5)}})),
			},
		})
		loop := bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: edgeNode(edge), Forward: true}}}})
		face := bornFace(&Face{n: topo.Face{Surface: surfacegeom.Plane{Frame: planeFrame}, Loops: []*topo.Loop{loopNode(loop)}}})

		_, err := st.faceHalfPlane(face)
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "NURBSCurve")
	})

	t.Run("FacetedCurve rim", func(t *testing.T) {
		edge := bornEdge(&Edge{
			n: topo.Edge{Curve: surfacegeom.FacetedCurve{}, Start: vertexNode(bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 0, 0)}})),
				End: vertexNode(bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 0, 5)}})),
			},
		})
		loop := bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: edgeNode(edge), Forward: true}}}})
		face := bornFace(&Face{n: topo.Face{Surface: surfacegeom.Plane{Frame: planeFrame}, Loops: []*topo.Loop{loopNode(loop)}}})

		_, err := st.faceHalfPlane(face)
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "FacetedCurve")
	})

	t.Run("Line3 rim resolves as before", func(t *testing.T) {
		// Regression: the vertex-only edge kind this audit has always
		// permitted keeps returning its agreed half-plane angle, never a
		// refusal, once the vertices themselves agree.
		edge := bornEdge(&Edge{
			n: topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexNode(bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 0, 0)}})),
				End: vertexNode(bornVertex(&Vertex{n: topo.Vertex{Position: r3.NewVec(1, 0, 5)}})),
			},
		})
		loop := bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: []topo.CoEdge{{Edge: edgeNode(edge), Forward: true}}}})
		face := bornFace(&Face{n: topo.Face{Surface: surfacegeom.Plane{Frame: planeFrame}, Loops: []*topo.Loop{loopNode(loop)}}})

		phi, err := st.faceHalfPlane(face)
		require.NoError(t, err)
		require.InDelta(t, 0, phi, 1e-12, "the vertices sit exactly on r0, phi = 0")
	})
}

// TestResolveToFaceRefusesStatedNormalDeparture pins that a stop face whose
// tag is exactly parallel to the sweep is still refused when the face states a
// departure of its true surface from that tag, since the perpendicular cap
// charges none of it.
func TestResolveToFaceRefusesStatedNormalDeparture(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	for _, tc := range []struct {
		name   string
		mutate func(f *Face)
	}{
		{name: "normal bound", mutate: func(f *Face) { f.n.NormalBound = 1e-12 }},
		{name: "denoted surface", mutate: func(f *Face) { f.n.Denoted = &surfacenormal.Revolved{Valid: true} }},
		{name: "no record", mutate: func(f *Face) { f.n.CapRecord = nil }},
		{name: "none", mutate: func(*Face) {}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := bornDocument(&Document{})
			host, err := evalPrism(doc, 1, prismPayload{Payload: prismrecord.Payload{
				Profile: profileRecord{Outer: synthRectLoop(0, 0, 1, 1)},
				Frame:   frame,
				Z0:      10,
				Z1:      20,
				Xform:   r3.Identity(),
			}}, freeform.NewFreeformWork())
			require.NoError(t, err)
			doc.n.LiveSet = []*topo.Body{bodyNode(host)}
			ref := Faces(FaceCreatedBy(featureRefOf(provenance.Ref{Producer: 1, Role: provenance.RoleCapEnd})))
			faces, err := ref.SelectFaces(host)
			require.NoError(t, err)
			require.Len(t, faces, 1)
			tc.mutate(faces[0])

			stop, _, _, err := doc.resolveToFace(ToFace{Body: host, Face: ref}, frame, 1, "a to-face extent")
			if tc.name == "none" {
				require.NoError(t, err)
				require.Equal(t, 20.0, stop)
				return
			}
			require.ErrorIs(t, err, ErrUnsupported)
		})
	}
}

// TestResolveToFaceUsesSelectedCapAxialDelta keeps a computed end's bound on
// that cap only. A later ToFace against the exact opposite cap must remain
// exact, while selecting the computed cap must retain its bound.
func TestResolveToFaceUsesSelectedCapAxialDelta(t *testing.T) {
	t.Parallel()
	frame := canonicalPrismFrame(t)
	doc := bornDocument(&Document{})
	host, err := evalPrism(doc, 1, prismPayload{Payload: prismrecord.Payload{
		Profile: profileRecord{Outer: synthRectLoop(0, 0, 1, 1)},
		Frame:   frame,
		Z0:      10,
		Z1:      20,
		Z1Delta: 0.25,
		Xform:   r3.Identity(),
	}}, freeform.NewFreeformWork())
	require.NoError(t, err)
	doc.n.LiveSet = []*topo.Body{bodyNode(host)}

	for _, tc := range []struct {
		name  string
		role  string
		stop  float64
		delta float64
	}{
		{name: "exact start cap", role: provenance.RoleCapStart, stop: 10, delta: 0},
		{name: "computed end cap", role: provenance.RoleCapEnd, stop: 20, delta: 0.25},
	} {
		t.Run(tc.name, func(t *testing.T) {
			stop, delta, _, err := doc.resolveToFace(ToFace{
				Body: host,
				Face: Faces(FaceCreatedBy(featureRefOf(provenance.Ref{Producer: 1, Role: tc.role}))),
			}, frame, 1, "a to-face extent")
			require.NoError(t, err)
			require.Equal(t, tc.stop, stop)
			if tc.delta == 0 {
				require.Zero(t, delta)
				return
			}
			require.GreaterOrEqual(t, delta, tc.delta)
		})
	}
}

// TestAngularStopEdgeNamesPublicCurveType pins that the refusal text keeps
// the public variant name although the edge node holds the internal twin.
func TestAngularStopEdgeNamesPublicCurveType(t *testing.T) {
	e := bornEdge(&Edge{n: topo.Edge{Curve: surfacegeom.Ellipse3{}}})
	require.Equal(t, "decad.Ellipse3", angularStopEdge(e).CurveType)
}

// genericPlateHost evaluates a 20×20 plate on the frame f, extruded from level
// 0 to z1, and returns the host document, the body and the far cap selector.
func genericPlateHost(t *testing.T, f r3.Frame, z1 float64) (*Document, *Body, *FaceQuery) {
	t.Helper()
	doc := bornDocument(&Document{})
	host, err := evalPrism(doc, 1, prismPayload{Payload: prismrecord.Payload{
		Profile: profileRecord{Outer: synthRectLoop(0, 0, 20, 20)},
		Frame:   f,
		Z0:      0,
		Z1:      z1,
		Xform:   r3.Identity(),
	}}, freeform.NewFreeformWork())
	require.NoError(t, err)
	doc.n.LiveSet = []*topo.Body{bodyNode(host)}
	return doc, host, Faces(FaceCreatedBy(featureRefOf(provenance.Ref{Producer: 1, Role: provenance.RoleCapEnd})))
}

func genericPlateFrames(t *testing.T) (f, flipped, skewed, shifted r3.Frame) {
	t.Helper()
	mk := func(o, u, v r3.Vec) r3.Frame {
		fr, err := r3.NewFrame(o, u, v)
		require.NoError(t, err)
		return fr
	}
	f = mk(r3.Vec{}, r3.NewVec(1, 1, 0), r3.NewVec(-1, 1, 1))
	flipped = mk(f.Origin(), f.V(), f.U().Scale(-1))
	skewed = mk(f.Origin(), f.U().Add(f.V()), f.V().Sub(f.U()))
	shifted = mk(f.Origin().Add(f.U().Scale(1e6)), f.U(), f.V())
	return f, flipped, skewed, shifted
}

// TestResolveToFaceChargesDenotedPlaneOnGenericFrame pins that the stop's
// published displacement covers its exact distance from the plane the stop
// cap's record denotes, on a generic frame whose held tag origin and normal are
// rounded. Each case is a sweep frame whose plane is an exact level set of the
// plate frame's plane, so the stop is the single level
// c = m·(P − O')/(m·N') with m = U×V, P = O + z·N, computed here in rationals.
func TestResolveToFaceChargesDenotedPlaneOnGenericFrame(t *testing.T) {
	t.Parallel()
	f, flipped, _, shifted := genericPlateFrames(t)
	ratVec := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
	}
	dot := func(a, b [3]*big.Rat) *big.Rat {
		s := new(big.Rat)
		for i := range a {
			s.Add(s, new(big.Rat).Mul(a[i], b[i]))
		}
		return s
	}
	for _, tc := range []struct {
		name  string
		z1    float64
		sweep r3.Frame
	}{
		{name: "same frame at 1e3", z1: 1e3, sweep: f},
		{name: "flipped axes at 1e3", z1: 1e3, sweep: flipped},
		{name: "flipped axes at 1e9", z1: 1e9, sweep: flipped},
		{name: "origin 1e6 along U at 1e6", z1: 1e6, sweep: shifted},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, host, ref := genericPlateHost(t, f, tc.z1)
			stop, delta, _, err := doc.resolveToFace(ToFace{Body: host, Face: ref}, tc.sweep, 1, "a to-face extent")
			require.NoError(t, err)

			u, v, n, o := ratVec(f.U()), ratVec(f.V()), ratVec(f.N()), ratVec(f.Origin())
			m := [3]*big.Rat{}
			for i := range m {
				j, k := (i+1)%3, (i+2)%3
				m[i] = new(big.Rat).Sub(new(big.Rat).Mul(u[j], v[k]), new(big.Rat).Mul(u[k], v[j]))
			}
			z := new(big.Rat).SetFloat64(tc.z1)
			so, sn := ratVec(tc.sweep.Origin()), ratVec(tc.sweep.N())
			var rel [3]*big.Rat
			for i := range rel {
				p := new(big.Rat).Add(o[i], new(big.Rat).Mul(z, n[i]))
				rel[i] = p.Sub(p, so[i])
			}
			c := new(big.Rat).Quo(dot(m, rel), dot(m, sn))
			gap := new(big.Rat).Sub(new(big.Rat).SetFloat64(stop), c)
			gap.Abs(gap)
			t.Logf("stop %.17g, exact level %s, gap %s, published delta %g", stop, c.FloatString(25), gap.FloatString(25), delta)
			require.LessOrEqual(t, gap.Cmp(new(big.Rat).SetFloat64(delta)), 0,
				"the published displacement must cover the stop's exact distance from the denoted level")
		})
	}
}

// TestResolveToFaceRefusesFootprintDependentStop pins that a stop cap whose
// denoted plane is not an exact level set of the sweep is refused. The sweep
// frame's held normal is exactly parallel to the plate cap's held normal, yet
// its axes are a rounding step off the plate frame's, so the cap's denoted
// plane meets the sweep at a level that varies across a distant footprint.
func TestResolveToFaceRefusesFootprintDependentStop(t *testing.T) {
	t.Parallel()
	f, _, skewed, _ := genericPlateFrames(t)
	doc, host, ref := genericPlateHost(t, f, 10)
	face, err := ref.SelectFaces(host)
	require.NoError(t, err)
	require.Len(t, face, 1)
	pl, ok := face[0].Surface().(Plane)
	require.True(t, ok)
	require.True(t, extent.ExactlyParallel(pl.Frame.N(), skewed.N()), "the held normals must pass the exact-parallel gate")

	stop, _, producer, err := doc.resolveToFace(ToFace{Body: host, Face: ref}, skewed, 1, "a to-face extent")
	require.ErrorIs(t, err, ErrUnsupported)
	require.Zero(t, stop)
	require.Zero(t, producer)
}
