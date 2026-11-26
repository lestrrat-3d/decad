package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// featureCommand is one sketch-fed feature command built over a fresh fixture
// whose result is known. check asserts the computed geometry of the result.
//
// sweep and stride tune the mid-build cancellation sweep only
// (TestFeatureCommandsReturnMidBuildCancellationByIdentity). sweep, when set,
// builds a smaller fixture on the same build route once and returns a function
// that makes a fresh command over it, so a canceled run, which leaves the
// document unchanged, costs one command and no fixture. stride moves the cancel
// that many checks later per run, where one check at a time would cost minutes.
type featureCommand struct {
	name   string
	kind   decad.OperationKind
	build  func(t *testing.T) (do func(context.Context) (*decad.Body, error), doc *decad.Document)
	check  func(t *testing.T, got *decad.Body)
	sweep  func(t *testing.T) (next func() func(context.Context) (*decad.Body, error), doc *decad.Document)
	stride int64
}

func requireFeatureVolume(t *testing.T, b *decad.Body, want float64) {
	t.Helper()
	v, err := b.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+1e-6)
}

func featureCommands() []featureCommand {
	mm := units.Millimeters
	return []featureCommand{
		{name: "Extrude", kind: decad.OperationExtrude,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				s, p := plateSketch(t)
				doc := decad.New()
				return decad.Extrude(doc, s, p, decad.Distance{D: mm(5), Dir: decad.Along}).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireFeatureVolume(t, got, 100*60*5) }},
		{name: "Revolve", kind: decad.OperationRevolve,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				s, p := revolveRectangle(t)
				doc := decad.New()
				return decad.Revolve(doc, s, p, decad.SketchLine{End: decad.Point2{U: 1}}, decad.FullRevolution{}).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireFeatureVolume(t, got, math.Pi*(15*15-5*5)*10) }},
		{name: "Sweep", kind: decad.OperationSweep,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				s, p := plateSketch(t)
				doc := decad.New()
				return decad.Sweep(doc, s, p, sweepLinePath(t)).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireFeatureVolume(t, got, 100*60*10) }},
		{name: "Coil", kind: decad.OperationCoil, stride: 145,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				s, p := coilSketch(t, [][2]float64{{2, 0}, {3, 0}, {3, 1}, {2, 1}})
				doc := decad.New()
				return decad.Coil(doc, s, p, coilAxis(), mm(1.5), units.Scalar(2)).Do, doc
			},
			// One turn polls about half as often as two on the same route.
			sweep: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
				s, p := coilSketch(t, [][2]float64{{2, 0}, {3, 0}, {3, 1}, {2, 1}})
				doc := decad.New()
				return func() func(context.Context) (*decad.Body, error) {
					return decad.Coil(doc, s, p, coilAxis(), mm(1.5), units.Scalar(1)).Do
				}, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireFeatureVolume(t, got, 10*math.Pi) }},
		{name: "Loft", kind: decad.OperationLoft,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				s0, p0, s1, p1 := loftSquares(t, 20, 20)
				doc := decad.New()
				return decad.Loft(doc, s0, p0, s1, p1).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireFeatureVolume(t, got, 40*40*10) }},
		{name: "LoftChain", kind: decad.OperationLoftChain,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				pts := [][2]float64{{0, 0}, {10, 0}}
				s0, c0, s1, c1 := chainLoftPair(t, 6, pts, pts)
				doc := decad.New()
				return decad.LoftChain(doc, s0, c0, s1, c1).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) {
				require.Equal(t, decad.BodySheet, got.Kind())
				box, err := got.Bounds()
				require.NoError(t, err)
				require.InDelta(t, 6.0, box.Max.Z-box.Min.Z, 1e-9)
			}},
		{name: "LoftFromPoint", kind: decad.OperationLoftFromPoint,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				s, p := pointLoftSquare(t, sketch.NewWorld(), 3)
				doc := decad.New()
				return decad.LoftFromPoint(doc, r3.Vec{}, s, p).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireFeatureVolume(t, got, 4*4*3/3.0) }},
		{name: "LoftSections", kind: decad.OperationLoftSections,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				sections := threeSquareSections(t, [3]float64{10, 10, 10}, [3]float64{0, 5, 10})
				doc := decad.New()
				return decad.LoftSections(doc, sections[:]...).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireFeatureVolume(t, got, 20*20*10) }},
		loftFacesFeature(),
		loftFacesToConeFeature(),
		{name: "PatchProfile", kind: decad.OperationPatchProfile,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				s, p := plateSketch(t)
				doc := decad.New()
				return decad.PatchProfile(doc, s, p).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) {
				require.Equal(t, decad.BodySheet, got.Kind())
				area, err := got.Area()
				require.NoError(t, err)
				require.InDelta(t, 100*60, area.Value.Base(), area.Bound.Base()+1e-6)
			}},
		{name: "Emboss", kind: decad.OperationEmboss, stride: 7,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, plate := embossPlate(t)
				s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 16})
				return decad.Emboss(plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, mm(2)).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireFeatureVolume(t, got, embossPlateVolume+10*6*2) }},
		{name: "WebPath", kind: decad.OperationWeb, stride: 13,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc, part := ribPart(t)
				s, lines := ribSketch(t, [4]float64{3, 20, 37, 20})
				return decad.WebPath(part, s, lines, mm(2), decad.Distance{D: mm(11), Dir: decad.Along}).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireFeatureVolume(t, got, ribPartVolume+30*2*10) }},
		{name: "CutExtrude", kind: decad.OperationCutExtrude, stride: 7,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc := decad.New()
				plate := boxBody(t, doc, -20, -20, 20, 20, 5)
				s := cutProfileSketch(t, [4]float64{-10, -10, -6, -6})
				return decad.CutExtrude(plate, s, s.Profiles(), decad.Distance{D: mm(3), Dir: decad.Along}).Do, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireFeatureVolume(t, got, 40*40*5-4*4*3) }},
		{name: "CutRevolve", kind: decad.OperationCutRevolve, stride: 172,
			build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
				doc := decad.New()
				plate := boxBody(t, doc, -20, -20, 20, 20, 5)
				s := cutProfileSketch(t, [4]float64{3, 1, 5, 3})
				axis := decad.SketchLine{End: decad.Point2{V: 1}}
				return decad.CutRevolve(plate, s, s.Profiles(), axis, decad.FullRevolution{}).Do, doc
			},
			// A narrower groove on the same full-revolution route.
			sweep: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
				doc := decad.New()
				plate := boxBody(t, doc, -20, -20, 20, 20, 5)
				s := cutProfileSketch(t, [4]float64{3, 1, 4, 2})
				axis := decad.SketchLine{End: decad.Point2{V: 1}}
				return func() func(context.Context) (*decad.Body, error) {
					return decad.CutRevolve(plate, s, s.Profiles(), axis, decad.FullRevolution{}).Do
				}, doc
			},
			check: func(t *testing.T, got *decad.Body) { requireFeatureVolume(t, got, 40*40*5-16*math.Pi) }},
	}
}

// loftFacesFeature is LoftFaces through the toe cap and the heel caps of two
// slabs of one split point loft, whose volume is the sum of the slabs'.
func loftFacesFeature() featureCommand {
	var want, allowance float64 // set by build and read by check on one goroutine.
	return featureCommand{name: "LoftFaces", kind: decad.OperationLoftFaces, stride: 27,
		build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
			doc := decad.New()
			kept := splitSlabs(t, doc)
			want, allowance = 0, 0
			for _, slab := range kept {
				v, err := slab.Volume()
				require.NoError(t, err)
				want += v.Value.Base()
				allowance += v.Bound.Base()
			}
			return decad.LoftFaces(slabLoftSections(t, kept)).Do, doc
		},
		sweep: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
			doc := decad.New()
			sections := slabLoftSections(t, splitSlabs(t, doc))
			return func() func(context.Context) (*decad.Body, error) { return decad.LoftFaces(sections).Do }, doc
		},
		check: func(t *testing.T, got *decad.Body) {
			v, err := got.Volume()
			require.NoError(t, err)
			require.Positive(t, want)
			require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+allowance+1e-9)
		}}
}

// loftFacesToConeFeature is LoftFacesToCone over a slab of a pointed solid and
// a cone tangent to the slab's source plane at its apex.
func loftFacesToConeFeature() featureCommand {
	return featureCommand{name: "LoftFacesToCone", kind: decad.OperationLoftFaces, stride: 9,
		build: func(t *testing.T) (func(context.Context) (*decad.Body, error), *decad.Document) {
			doc := decad.New()
			f := newConeTangentLoft(t, doc)
			return decad.LoftFacesToCone([]*decad.Face{f.toe, f.heel}, f.cone, f.radial).Do, doc
		},
		sweep: func(t *testing.T) (func() func(context.Context) (*decad.Body, error), *decad.Document) {
			doc := decad.New()
			f := newConeTangentLoft(t, doc)
			return func() func(context.Context) (*decad.Body, error) {
				return decad.LoftFacesToCone([]*decad.Face{f.toe, f.heel}, f.cone, f.radial).Do
			}, doc
		},
		check: func(t *testing.T, got *decad.Body) {
			require.True(t, got.IsSolid())
			// examples/loft_faces_cone_tangent_example_test.go prints this fixture's volume, 0.0033 mm^3.
			v, err := got.Volume()
			require.NoError(t, err)
			require.InDelta(t, 0.0033, v.Value.Base(), v.Bound.Base()+5e-5)
		}}
}

// revolveRectangle is the rectangle (0, 5)-(10, 15) of the XY plane, which a
// revolve about the u axis spins into a tube.
func revolveRectangle(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 5, 10, 15)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

func TestFeatureCommandsRunOnceAndConsume(t *testing.T) {
	t.Parallel()
	for _, tc := range featureCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			do, doc := tc.build(t)
			got, err := do(t.Context())
			require.NoError(t, err)
			tc.check(t, got)
			requireConsumed(t, do, doc, tc.kind)
		})
	}
}

// requireConsumed runs do again and requires the tagged ErrConsumed refusal
// that changes nothing in doc.
func requireConsumed(t *testing.T, do func(context.Context) (*decad.Body, error), doc *decad.Document, kind decad.OperationKind) {
	t.Helper()
	bodies := doc.Bodies()
	again, err := do(t.Context())
	require.ErrorIs(t, err, decad.ErrConsumed)
	require.Nil(t, again)
	first := requireOperationKind(t, err, kind)[0]
	require.Equal(t, decad.OpDiagInputRejected, first.Code)
	require.Equal(t, decad.OperationStageInput, first.Stage)
	require.Empty(t, first.Subject.Name)
	require.Equal(t, bodies, doc.Bodies(), "a consumed command changes nothing")
}

func TestFeatureCommandsRefuseNilContext(t *testing.T) {
	t.Parallel()
	for _, tc := range featureCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			do, doc := tc.build(t)
			before := doc.Bodies()
			var nilCtx context.Context
			got, err := do(nilCtx)
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.Nil(t, got)
			requireOperationKind(t, err, tc.kind)
			require.Equal(t, before, doc.Bodies())

			_, err = do(t.Context())
			require.ErrorIs(t, err, decad.ErrConsumed, "the refused Do consumed the command")
		})
	}
}

func TestFeatureCommandsReturnCancellationByIdentity(t *testing.T) {
	t.Parallel()
	for _, tc := range featureCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			do, doc := tc.build(t)
			before := doc.Bodies()
			ctx, cancel := context.WithCancel(t.Context())
			cancel()
			_, err := do(ctx)
			require.Equal(t, context.Canceled, err)
			require.Equal(t, before, doc.Bodies())
		})
	}
}

// A cancel that lands inside the build returns context.Canceled by identity,
// not a sentinel wrapping it (agent-docs/command-api-design.md section 3). The
// sweep moves the cancel one check later each run until the command finishes,
// or by the member's stride where its build polls so often that one check at a
// time would cost minutes (agent-docs/command-api-design.md section 14). The
// sweep ends at the first run that succeeds. A member with a sweep fixture
// reuses it across the canceled runs, each of which leaves the document unchanged.
func TestFeatureCommandsReturnMidBuildCancellationByIdentity(t *testing.T) {
	t.Parallel()
	const maxChecks = 6000
	for _, tc := range featureCommands() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			step := max(tc.stride, 1)
			fresh := func() (func(context.Context) (*decad.Body, error), *decad.Document) { return tc.build(t) }
			if tc.sweep != nil {
				next, fixture := tc.sweep(t)
				fresh = func() (func(context.Context) (*decad.Body, error), *decad.Document) { return next(), fixture }
			}
			for n := int64(0); n < maxChecks; n += step {
				do, doc := fresh()
				before := doc.Bodies()
				got, err := do(newCancelAfter(t.Context(), n))
				if err == nil {
					require.NotNil(t, got)
					require.Greater(t, n, int64(1), "the cancel never reached the build")
					return
				}
				require.Equal(t, context.Canceled, err, "cancel after %d checks", n)
				require.Nil(t, got)
				require.Equal(t, before, doc.Bodies(), "cancel after %d checks leaves the document unchanged", n)
			}
			require.FailNow(t, "the command still polled the context", "after %d checks", maxChecks)
		})
	}
}

// The constructor parameters that name a recorded fault.
const (
	subjectCone   = "cone"
	subjectTarget = "target"
)

// A nil operand is recorded by the constructor and named after the constructor
// parameter, and a nil ctx is still refused ahead of it.
func TestFeatureConstructorsRecordNilOperands(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	ext := decad.Distance{D: units.Millimeters(5), Dir: decad.Along}
	var nilDoc *decad.Document
	var nilSketch *sketch.Sketch
	var nilProfile *sketch.Profile
	var nilChain *sketch.Chain
	var nilBody *decad.Body
	var nilPath *decad.Path
	doc := decad.New()
	for _, tc := range []struct {
		name    string
		kind    decad.OperationKind
		subject string
		do      func(context.Context) (*decad.Body, error)
	}{
		{"Extrude document", decad.OperationExtrude, "d", decad.Extrude(nilDoc, s, p, ext).Do},
		{"Extrude sketch", decad.OperationExtrude, "s", decad.Extrude(doc, nilSketch, p, ext).Do},
		{"Extrude profile", decad.OperationExtrude, "sec", decad.Extrude(doc, s, nilProfile, ext).Do},
		{"Extrude chain", decad.OperationExtrude, "sec", decad.Extrude(doc, s, nilChain, ext).Do},
		{"Revolve document", decad.OperationRevolve, "d", decad.Revolve(nilDoc, s, p, decad.SketchLine{}, decad.FullRevolution{}).Do},
		{"Revolve profile", decad.OperationRevolve, "sec", decad.Revolve(doc, s, nilProfile, decad.SketchLine{}, decad.FullRevolution{}).Do},
		{"Revolve chain", decad.OperationRevolve, "sec", decad.Revolve(doc, s, nilChain, decad.SketchLine{}, decad.FullRevolution{}).Do},
		{"Sweep document", decad.OperationSweep, "d", decad.Sweep(nilDoc, s, p, sweepLinePath(t)).Do},
		{"Sweep path", decad.OperationSweep, "path", decad.Sweep(doc, s, p, nilPath).Do},
		{"Coil sketch", decad.OperationCoil, "s", decad.Coil(doc, nilSketch, p, coilAxis(), units.Millimeters(1), units.Scalar(1)).Do},
		{"Coil profile", decad.OperationCoil, "p", decad.Coil(doc, s, nilProfile, coilAxis(), units.Millimeters(1), units.Scalar(1)).Do},
		{"Loft document", decad.OperationLoft, "d", decad.Loft(nilDoc, s, p, s, p).Do},
		{"Loft second sketch", decad.OperationLoft, "s1", decad.Loft(doc, s, p, nilSketch, p).Do},
		{"Loft second profile", decad.OperationLoft, "p1", decad.Loft(doc, s, p, s, nilProfile).Do},
		{"LoftChain first chain", decad.OperationLoftChain, "c0", decad.LoftChain(doc, s, nilChain, s, nilChain).Do},
		{"LoftFromPoint profile", decad.OperationLoftFromPoint, "p", decad.LoftFromPoint(doc, r3.Vec{}, s, nilProfile).Do},
		{"LoftSections too few", decad.OperationLoftSections, "sections", decad.LoftSections(doc).Do},
		{"LoftSections document", decad.OperationLoftSections, "d", decad.LoftSections(nilDoc).Do},
		{"LoftFaces too few", decad.OperationLoftFaces, "sections", decad.LoftFaces(nil).Do},
		{"LoftFacesToCone nil cone", decad.OperationLoftFaces, subjectCone, decad.LoftFacesToCone(nil, nilBody, r3.NewVec(0, 1, 0)).Do},
		{"PatchProfile document", decad.OperationPatchProfile, "d", decad.PatchProfile(nilDoc, s, p).Do},
		{"Emboss body", decad.OperationEmboss, "body", decad.Emboss(nilBody, s, nil, nil, decad.EmbossRaised, units.Millimeters(1)).Do},
		{"WebPath body", decad.OperationWeb, "body", decad.WebPath(nilBody, s, nil, units.Millimeters(1), ext).Do},
		{"CutExtrude target", decad.OperationCutExtrude, subjectTarget, decad.CutExtrude(nilBody, s, nil, ext).Do},
		{"CutRevolve target", decad.OperationCutRevolve, subjectTarget, decad.CutRevolve(nilBody, s, nil, decad.SketchLine{}, decad.FullRevolution{}).Do},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, err := tc.do(t.Context())
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.Nil(t, got)
			diagnostics := requireOperationKind(t, err, tc.kind)
			require.Equal(t, tc.subject, diagnostics[0].Subject.Name)
		})
	}
	t.Run("nil context first", func(t *testing.T) {
		t.Parallel()
		var nilCtx context.Context
		for _, tc := range []struct {
			name string
			kind decad.OperationKind
			do   func(context.Context) (*decad.Body, error)
		}{
			{"Extrude", decad.OperationExtrude, decad.Extrude(nilDoc, s, p, ext).Do},
			{"Coil", decad.OperationCoil,
				decad.Coil(doc, nilSketch, p, coilAxis(), units.Millimeters(1), units.Scalar(1)).Do},
			{"LoftFromPoint", decad.OperationLoftFromPoint, decad.LoftFromPoint(doc, r3.Vec{}, s, nilProfile).Do},
			{"LoftSections", decad.OperationLoftSections, decad.LoftSections(nilDoc).Do},
			{"LoftFaces", decad.OperationLoftFaces, decad.LoftFaces(nil).Do},
			{"Emboss", decad.OperationEmboss,
				decad.Emboss(nilBody, s, nil, nil, decad.EmbossRaised, units.Millimeters(1)).Do},
			{"WebPath", decad.OperationWeb, decad.WebPath(nilBody, s, nil, units.Millimeters(1), ext).Do},
			{"CutExtrude", decad.OperationCutExtrude, decad.CutExtrude(nilBody, s, nil, ext).Do},
			{"CutRevolve", decad.OperationCutRevolve,
				decad.CutRevolve(nilBody, s, nil, decad.SketchLine{}, decad.FullRevolution{}).Do},
		} {
			_, err := tc.do(nilCtx)
			require.ErrorIs(t, err, decad.ErrDegenerate, tc.name)
			diagnostics := requireOperationKind(t, err, tc.kind)
			require.Empty(t, diagnostics[0].Subject.Name, "%s: a nil context is no fault of an operand", tc.name)
		}
	})
	t.Run("constructor fault before method fault", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Extrude(nilDoc, s, p, ext).Taper(units.Millimeters(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.NotErrorIs(t, err, decad.ErrUnitKind)
		diagnostics := requireOperationKind(t, err, decad.OperationExtrude)
		require.Equal(t, "d", diagnostics[0].Subject.Name)
	})
}

// A command built on a body that an earlier command retired reports
// ErrRetiredBody ahead of the fault a stored method value carries.
func TestFeatureCommandsRefuseRetiredBodiesFirst(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	doc, plate := embossPlate(t)
	s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 16})
	out, err := decad.Emboss(plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, mm(2)).Do(t.Context())
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	_, err = decad.Emboss(plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, mm(-2)).RimFillet(mm(0)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrRetiredBody, "Emboss reports the retired body before its rim fillet fault")

	ribDoc, part := ribPart(t)
	rs, lines := ribSketch(t, [4]float64{3, 20, 37, 20})
	ext := decad.Distance{D: mm(11), Dir: decad.Along}
	_, err = decad.WebPath(part, rs, lines, mm(2), ext).Do(t.Context())
	require.NoError(t, err)
	_, err = decad.WebPath(part, rs, lines, mm(-2), ext).Overrun(mm(-1)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrRetiredBody, "WebPath reports the retired body before its overrun fault")
	require.Len(t, ribDoc.Bodies(), 1)

	cutDoc := decad.New()
	target := boxBody(t, cutDoc, -20, -20, 20, 20, 5)
	cs := cutProfileSketch(t, [4]float64{-10, -10, -6, -6})
	cut, err := decad.CutExtrude(target, cs, cs.Profiles(), decad.Distance{D: mm(3), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	_, err = decad.CutExtrude(target, cs, cs.Profiles(), decad.Distance{D: mm(3), Dir: decad.Along}).
		Taper(mm(1)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrRetiredBody, "CutExtrude reports the retired target before its taper fault")
	require.Equal(t, []*decad.Body{cut}, cutDoc.Bodies())
}

// Each stored method fault is refused at its documented gate,
// relative to the gates around it, and names its method as the subject.
func TestFeatureCommandsRefuseMethodFaultsAtTheirGate(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	subject := func(t *testing.T, err error, kind decad.OperationKind) string {
		t.Helper()
		return requireOperationKind(t, err, kind)[0].Subject.Name
	}
	t.Run("Taper follows the profile seam and precedes the extent", func(t *testing.T) {
		t.Parallel()
		s, p := plateSketch(t)
		other, _ := plateSketch(t)
		_, err := decad.Extrude(decad.New(), other, p, decad.Distance{D: mm(5), Dir: decad.Along}).Taper(mm(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrForeignProfile, "the seam gate comes first")
		_, err = decad.Extrude(decad.New(), s, p, decad.Distance{D: mm(0), Dir: decad.Along}).Taper(mm(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnitKind, "the taper fault comes before the extent")
		require.Equal(t, "Taper", subject(t, err, decad.OperationExtrude))
	})
	t.Run("Taper on a chain is the chain refusal", func(t *testing.T) {
		t.Parallel()
		s, ch := lineChainSketch(t)
		_, err := decad.Extrude(decad.New(), s, ch, decad.Distance{D: mm(5), Dir: decad.Along}).Taper(mm(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.NotErrorIs(t, err, decad.ErrUnitKind)
	})
	t.Run("Twist and SectionScale precede the path build and the profile seam", func(t *testing.T) {
		t.Parallel()
		s, p := plateSketch(t)
		other, _ := plateSketch(t)
		path := sweepLinePath(t)
		_, err := decad.Sweep(decad.New(), other, p, path).Twist(mm(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnitKind)
		require.Equal(t, "Twist", subject(t, err, decad.OperationSweep))
		_, err = decad.Sweep(decad.New(), s, p, path).SectionScale(mm(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnitKind)
		require.Equal(t, "SectionScale", subject(t, err, decad.OperationSweep))
		_, err = decad.Sweep(decad.New(), s, p, path).SectionScale(units.Scalar(1), units.Scalar(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, "SectionScale", subject(t, err, decad.OperationSweep), "the count fault names the method")
		_, err = decad.Sweep(decad.New(), s, p, path).Twist(mm(1)).SectionScale(mm(1)).Do(t.Context())
		require.Equal(t, "SectionScale", subject(t, err, decad.OperationSweep), "the scale is refused before the twist")
	})
	t.Run("CircularCorrespondence follows the section count", func(t *testing.T) {
		t.Parallel()
		sections := threeSquareSections(t, [3]float64{10, 10, 10}, [3]float64{0, 5, 10})
		_, err := decad.LoftSections(decad.New(), sections[:]...).CircularCorrespondence(decad.CircularLoftCorrespondence{}).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, "CircularCorrespondence", subject(t, err, decad.OperationLoftSections))
		_, err = decad.LoftSections(decad.New(), sections[:2]...).CircularCorrespondence(decad.CircularLoftCorrespondence{}).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate, "the recorded section count comes first")
		require.Equal(t, "sections", subject(t, err, decad.OperationLoftSections))
	})
	t.Run("RimFillet follows the face gate and precedes the height", func(t *testing.T) {
		t.Parallel()
		doc, plate := embossPlate(t)
		s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 16})
		_, err := decad.Emboss(plate, s, profiles, nil, decad.EmbossRaised, mm(-2)).RimFillet(mm(0)).Do(t.Context())
		require.Error(t, err)
		require.NotErrorIs(t, err, decad.ErrNegativeMagnitude)
		_, err = decad.Emboss(plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, mm(-2)).RimFillet(mm(0)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.NotErrorIs(t, err, decad.ErrNegativeMagnitude, "the rim fillet fault comes before the height")
		require.Equal(t, "RimFillet", subject(t, err, decad.OperationEmboss))
		require.Equal(t, []*decad.Body{plate}, doc.Bodies())
	})
	t.Run("Draft of an emboss names the method", func(t *testing.T) {
		t.Parallel()
		doc, plate := embossPlate(t)
		s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 16})
		_, err := decad.Emboss(plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, mm(2)).Draft(mm(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnitKind)
		require.Equal(t, "Draft", subject(t, err, decad.OperationEmboss))
		require.Equal(t, []*decad.Body{plate}, doc.Bodies())
	})
	t.Run("Overrun follows the thickness and precedes the path", func(t *testing.T) {
		t.Parallel()
		doc, part := ribPart(t)
		s, lines := ribSketch(t, [4]float64{3, 20, 37, 20})
		ext := decad.Distance{D: mm(11), Dir: decad.Along}
		_, err := decad.WebPath(part, s, lines, units.Degrees(2), ext).Overrun(mm(-1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnitKind, "the thickness gate comes first")
		_, err = decad.WebPath(part, s, nil, mm(2), ext).Overrun(mm(-1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrNegativeMagnitude, "the overrun fault comes before the empty path")
		require.Equal(t, "Overrun", subject(t, err, decad.OperationWeb))
		require.Equal(t, []*decad.Body{part}, doc.Bodies())
	})
	t.Run("Draft of a web names the method", func(t *testing.T) {
		t.Parallel()
		doc, part := ribPart(t)
		s, lines := ribSketch(t, [4]float64{3, 20, 37, 20})
		_, err := decad.WebPath(part, s, lines, mm(2), decad.Distance{D: mm(11), Dir: decad.Along}).Draft(mm(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnitKind)
		require.Equal(t, "Draft", subject(t, err, decad.OperationWeb))
		require.Equal(t, []*decad.Body{part}, doc.Bodies())
	})
	t.Run("Taper of a cut follows the target gates", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		target := boxBody(t, doc, -20, -20, 20, 20, 5)
		cs := cutProfileSketch(t, [4]float64{-10, -10, -6, -6})
		_, err := decad.CutExtrude(target, cs, cs.Profiles(), decad.Distance{D: mm(3), Dir: decad.Along}).Taper(mm(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnitKind)
		require.Equal(t, "Taper", subject(t, err, decad.OperationCutExtrude))
		require.Equal(t, []*decad.Body{target}, doc.Bodies())
		_, err = decad.CutExtrude(target, cs, nil, decad.Distance{D: mm(3), Dir: decad.Along}).Taper(mm(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate, "the empty profile list comes before the taper")
		require.NotErrorIs(t, err, decad.ErrUnitKind)
	})
}

// A command copies its slices when it is built, so a later write to the
// caller's slice reaches nothing (rule A2).
func TestFeatureCommandsCopyTheirSlices(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	t.Run("Emboss profiles", func(t *testing.T) {
		t.Parallel()
		_, plate := embossPlate(t)
		s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 16})
		cmd := decad.Emboss(plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, mm(2))
		profiles[0] = nil
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		requireFeatureVolume(t, got, embossPlateVolume+10*6*2)
	})
	t.Run("WebPath entities", func(t *testing.T) {
		t.Parallel()
		_, part := ribPart(t)
		s, lines := ribSketch(t, [4]float64{3, 20, 37, 20})
		cmd := decad.WebPath(part, s, lines, mm(2), decad.Distance{D: mm(11), Dir: decad.Along})
		lines[0] = nil
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		requireFeatureVolume(t, got, ribPartVolume+30*2*10)
	})
	t.Run("CutExtrude profiles", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		target := boxBody(t, doc, -20, -20, 20, 20, 5)
		cs := cutProfileSketch(t, [4]float64{-10, -10, -6, -6})
		profiles := cs.Profiles()
		cmd := decad.CutExtrude(target, cs, profiles, decad.Distance{D: mm(3), Dir: decad.Along})
		profiles[0] = nil
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		requireFeatureVolume(t, got, 40*40*5-4*4*3)
	})
	t.Run("CutRevolve profiles", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		target := boxBody(t, doc, -20, -20, 20, 20, 5)
		cs := cutProfileSketch(t, [4]float64{3, 1, 5, 3})
		profiles := cs.Profiles()
		cmd := decad.CutRevolve(target, cs, profiles, decad.SketchLine{End: decad.Point2{V: 1}}, decad.FullRevolution{})
		profiles[0] = nil
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		requireFeatureVolume(t, got, 40*40*5-16*math.Pi)
	})
	t.Run("LoftSections sections", func(t *testing.T) {
		t.Parallel()
		sections := threeSquareSections(t, [3]float64{10, 10, 10}, [3]float64{0, 5, 10})
		cmd := decad.LoftSections(decad.New(), sections[:]...)
		sections[1] = decad.LoftSection{}
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		requireFeatureVolume(t, got, 20*20*10)
	})
	t.Run("LoftFaces sections", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		kept := splitSlabs(t, doc)
		sections := slabLoftSections(t, kept)
		cmd := decad.LoftFaces(sections)
		sections[0] = nil
		got, err := cmd.Do(t.Context())
		require.NoError(t, err, "the copy still holds the first cap")
		want := 0.0
		for _, slab := range kept {
			v, err := slab.Volume()
			require.NoError(t, err)
			want += v.Value.Base()
		}
		v, err := got.Volume()
		require.NoError(t, err)
		require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+1e-6*want)
	})
	t.Run("Loft alignment and splits", func(t *testing.T) {
		t.Parallel()
		s0, p0, s1, p1 := loftSquares(t, 20, 20)
		offsets := []int{0}
		cmd := decad.Loft(decad.New(), s0, p0, s1, p1).Alignment(offsets...)
		offsets[0] = 3 // a later write that would pair another vertex.
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		requireFeatureVolume(t, got, 40*40*10)
	})
}

// A command copies the selectors its sealed arguments hold when it is built, so
// a later change to the caller's query reaches nothing (rule A3).
func TestFeatureCommandsCopySelectorsInSealedArguments(t *testing.T) {
	t.Parallel()
	t.Run("ToFace of an Extrude", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		s, plateProf, pinProf := plateAndPin(t)
		plate, err := decad.Extrude(doc, s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}).Do(t.Context())
		require.NoError(t, err)
		q := capEndFace(plate)
		cmd := decad.Extrude(doc, s, pinProf, decad.ToFace{Body: plate, Face: q})
		q.Exactly(3) // a later change that would refuse the one cap.
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		requireFeatureVolume(t, got, 20*20*10)
	})
	t.Run("ToFace as a side of a TwoSided", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		s, plateProf, pinProf := plateAndPin(t)
		plate, err := decad.Extrude(doc, s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}).Do(t.Context())
		require.NoError(t, err)
		q := capEndFace(plate)
		cmd := decad.Extrude(doc, s, pinProf, decad.TwoSided{
			One: decad.ToFace{Body: plate, Face: q},
			Two: decad.DistanceSide{D: units.Millimeters(2)},
		})
		q.Exactly(3)
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		requireFeatureVolume(t, got, 20*20*12)
	})
	t.Run("face selector of an Emboss", func(t *testing.T) {
		t.Parallel()
		_, plate := embossPlate(t)
		s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 16})
		q := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate)))
		cmd := decad.Emboss(plate, s, profiles, q, decad.EmbossRaised, units.Millimeters(2))
		q.Exactly(3)
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		requireFeatureVolume(t, got, embossPlateVolume+10*6*2)
	})
}

// A later call of a method replaces the earlier one, its fault included
// (agent-docs/command-api-design.md rule M2).
func TestFeatureMethodsLastCallWins(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	t.Run("Extrude Taper", func(t *testing.T) {
		t.Parallel()
		s, p := plateSketch(t)
		ext := decad.Distance{D: mm(5), Dir: decad.Along}
		straight, err := decad.Extrude(decad.New(), s, p, ext).Taper(units.Degrees(10)).Taper(units.Degrees(0)).Do(t.Context())
		require.NoError(t, err)
		requireFeatureVolume(t, straight, 100*60*5)
		drafted, err := decad.Extrude(decad.New(), s, p, ext).Taper(units.Degrees(0)).Taper(units.Degrees(10)).Do(t.Context())
		require.NoError(t, err)
		v, err := drafted.Volume()
		require.NoError(t, err)
		require.Less(t, v.Value.Base(), 100.0*60*5, "the second taper narrows the body")
		// A refused taper is replaced, with its fault, by a later call, and the reverse refuses.
		replaced, err := decad.Extrude(decad.New(), s, p, ext).Taper(units.Degrees(math.NaN())).Taper(units.Degrees(0)).Do(t.Context())
		require.NoError(t, err, "a valid call replaces an invalid one")
		requireFeatureVolume(t, replaced, 100*60*5)
		_, err = decad.Extrude(decad.New(), s, p, ext).Taper(units.Degrees(0)).Taper(units.Degrees(math.NaN())).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrNotFinite, "an invalid call replaces a valid one")
	})
	t.Run("Extrude SurfaceResult is idempotent", func(t *testing.T) {
		t.Parallel()
		s, p := plateSketch(t)
		got, err := decad.Extrude(decad.New(), s, p, decad.Distance{D: mm(5), Dir: decad.Along}).
			SurfaceResult().SurfaceResult().Do(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.BodySheet, got.Kind())
	})
	t.Run("Revolve SurfaceResult", func(t *testing.T) {
		t.Parallel()
		s, p := revolveRectangle(t)
		got, err := decad.Revolve(decad.New(), s, p, decad.SketchLine{End: decad.Point2{U: 1}},
			decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along}).SurfaceResult().SurfaceResult().Do(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.BodySheet, got.Kind())
	})
	t.Run("Sweep SectionScale and MitredJoins", func(t *testing.T) {
		t.Parallel()
		s, p := squareSweepSketch(t, 2)
		ell := mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1))
		scale := func(factors ...float64) *decad.SweepCommand {
			return decad.Sweep(decad.New(), s, p, ell).MitredJoins().MitredJoins().SectionScale(scalars(factors...)...)
		}
		_, err := scale(0).SectionScale(units.Scalar(1)).Do(t.Context())
		require.NoError(t, err, "a valid call replaces an invalid one")
		_, err = decad.Sweep(decad.New(), s, p, ell).MitredJoins().SectionScale(units.Scalar(1)).SectionScale(units.Scalar(0)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrDegenerate, "an invalid call replaces a valid one")
	})
	t.Run("Sweep Twist", func(t *testing.T) {
		t.Parallel()
		s, p := plateSketch(t)
		path := sweepLinePath(t)
		got, err := decad.Sweep(decad.New(), s, p, path).Twist(mm(1)).Twist(units.Degrees(0)).Do(t.Context())
		require.NoError(t, err, "a valid call replaces an invalid one")
		requireFeatureVolume(t, got, 100*60*10)
		_, err = decad.Sweep(decad.New(), s, p, path).Twist(units.Degrees(0)).Twist(mm(1)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnitKind, "an invalid call replaces a valid one")
	})
	t.Run("Coil LeftHand", func(t *testing.T) {
		t.Parallel()
		build := func(left int) *decad.Body {
			s, p := coilSketch(t, [][2]float64{{2, 0}, {3, 0}, {3, 1}, {2, 1}})
			cmd := decad.Coil(decad.New(), s, p, coilAxis(), mm(1.5), units.Scalar(1.25))
			for range left {
				cmd.LeftHand()
			}
			got, err := cmd.Do(t.Context())
			require.NoError(t, err)
			return got
		}
		centroidZ := func(b *decad.Body) float64 {
			c, err := b.Centroid()
			require.NoError(t, err)
			return c.Value.Z
		}
		right, once, twice := centroidZ(build(0)), centroidZ(build(1)), centroidZ(build(2))
		require.InDelta(t, once, twice, 1e-9, "a second call is the first")
		require.InDelta(t, -right, once, 1e-6, "left-handed mirrors the right-handed coil across the axis plane")
		require.Greater(t, math.Abs(right), 1e-3)
	})
	t.Run("Loft Alignment", func(t *testing.T) {
		t.Parallel()
		s0, p0, s1, p1 := loftSquares(t, 20, 20)
		// Two offsets name a hole loop the profiles do not have.
		got, err := decad.Loft(decad.New(), s0, p0, s1, p1).Alignment(0, 0).Alignment(0).Do(t.Context())
		require.NoError(t, err)
		requireFeatureVolume(t, got, 40*40*10)
		doc := decad.New()
		refused, err := decad.Loft(doc, s0, p0, s1, p1).Alignment(0).Alignment(0, 0).Do(t.Context())
		require.Nil(t, refused)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Empty(t, doc.Bodies())
	})
	t.Run("LoftSections CircularCorrespondence", func(t *testing.T) {
		t.Parallel()
		sections := threeSquareSections(t, [3]float64{10, 10, 10}, [3]float64{0, 5, 10})
		bad := decad.CircularLoftCorrespondence{}
		_, err := decad.LoftSections(decad.New(), sections[:]...).CircularCorrespondence(bad).CircularCorrespondence(bad).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
	t.Run("Emboss RimFillet and Draft", func(t *testing.T) {
		t.Parallel()
		_, plate := embossPlate(t)
		s, profiles := embossSketch(t, 10, [4]float64{10, 10, 20, 16})
		got, err := decad.Emboss(plate, s, profiles, embossFace(true, plate), decad.EmbossRaised, mm(2)).
			RimFillet(mm(0)).RimFillet(mm(0.5)).Draft(mm(1)).Draft(units.Degrees(0)).Do(t.Context())
		require.NoError(t, err)
		v, err := got.Volume()
		require.NoError(t, err)
		require.Less(t, v.Value.Base(), float64(embossPlateVolume+10*6*2), "the rim fillet removes material from the relief")
		require.Greater(t, v.Value.Base(), float64(embossPlateVolume))
	})
	t.Run("WebPath Draft and Overrun", func(t *testing.T) {
		t.Parallel()
		_, part := ribPart(t)
		s, lines := ribSketch(t, [4]float64{3, 20, 37, 20})
		got, err := decad.WebPath(part, s, lines, mm(2), decad.Distance{D: mm(11), Dir: decad.Along}).
			Draft(mm(1)).Draft(units.Degrees(0)).Overrun(mm(-1)).Overrun(mm(0)).Do(t.Context())
		require.NoError(t, err)
		requireFeatureVolume(t, got, ribPartVolume+30*2*10)
	})
	t.Run("CutExtrude Taper", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		target := boxBody(t, doc, -20, -20, 20, 20, 5)
		cs := cutProfileSketch(t, [4]float64{-10, -10, -6, -6})
		got, err := decad.CutExtrude(target, cs, cs.Profiles(), decad.Distance{D: mm(3), Dir: decad.Along}).
			Taper(mm(1)).Taper(units.Degrees(0)).Do(t.Context())
		require.NoError(t, err)
		decadtest.MeasuresVolume(t, got, units.CubicMillimeters(40*40*5-4*4*3))
	})
}
