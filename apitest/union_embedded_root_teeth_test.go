package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// bevelMember is the blank, cones and one tooth of a straight bevel gear
// member (module 4, 90 degree shaft): a straight bevel pair, 32 teeth meshing
// with 16. The flank
// is the left involute flank in the tooth's back-cone plane; the right flank
// mirrors it. When the involute base circle lies inside the root circle
// (embedded), Sketch trims each flank at the root circle and the tooth root is
// a fragment of a Circle; otherwise short lines join the flanks to a root arc.
type bevelMember struct {
	blank                [][2]float64
	toeApex, heelApex    float64
	coneAngle, coneDist  float64
	rootRadius, baseRoot float64
	embedded             bool
	flank                [][2]float64
}

var (
	// 32 teeth meshing with 16: embedded root.
	bevel32 = bevelMember{
		blank: [][2]float64{{32, 0}, {48, 0}, {48, 56},
			{36.47213595499959, 61.76393202250021}, {29.542430123549664, 50.028784938225165}, {32, 48.8}},
		toeApex: 129.59999999999997, heelApex: 159.99999999999997,
		coneAngle: 1.1071487177940904, coneDist: 71.554175279993274,
		rootRadius: 137.65835055998653, embedded: true,
		flank: [][2]float64{{-134.38653173247118, -4.9553190064669028}, {-135.29065544982657, -4.9186846732736802}, {-136.19683200656632, -4.8234350222981286}, {-137.10422558521304, -4.6892870949145484}, {-138.0124734766982, -4.5231199435221106}, {-138.92132258000498, -4.3288328521962331}, {-139.83056811149615, -4.1090077362395432}, {-140.74003162727976, -3.8655053864492861}, {-141.64955084904307, -3.5997416080636357}, {-142.55897421189957, -3.3128353009844367}, {-143.46815764510174, -3.0056959860756556}, {-144.37696253425341, -2.6790792902035374}, {-145.28525436192405, -2.3336240338416192}, {-146.19290176287313, -1.9698780751709302}, {-147.09977584549117, -1.588316935043675}},
	}
	// 16 teeth meshing with 32: root arc joined by lines, and tip nodes that
	// meet the heel cone to within rounding.
	bevel16 = bevelMember{
		blank: [][2]float64{{64.00000000000001, 0}, {72.00000000000001, 0}, {72.00000000000001, 16},
			{66.2360679774998, 27.52786404500042}, {53.65121506177484, 22.297569876450343}, {64.00000000000001, 1.6000000000000014}},
		toeApex: 64.800000000000011, heelApex: 80,
		coneAngle: 0.46364760900080609, coneDist: 71.554175279993274,
		rootRadius: 30.32708763999663, baseRoot: 33.619465248515567,
		flank: [][2]float64{{-33.44227368820534, -3.4471399970576404}, {-33.884650502591988, -3.4447154858957751}, {-34.330982705943377, -3.401954280932368}, {-34.779717699027586, -3.3319463446116293}, {-35.230191200543715, -3.2391545163774951}, {-35.681942464990712, -3.126058449816747}, {-36.134599494228866, -2.9942687847488663}, {-36.587838082477035, -2.844925030556134}, {-37.04136295929959, -2.6788790697216607}, {-37.494897746447037, -2.4967931568905781}, {-37.948179070179151, -2.2991976062985606}, {-38.4009528660116, -2.0865272119604001}, {-38.85297193867703, -1.859145492774249}, {-39.303994287010795, -1.6173615189281181}, {-39.753781918583442, -1.3614419890990246}},
	}
)

func bevelDimensioned(s *sketch.Sketch, x, y float64) *sketch.Point {
	p := s.CreatePoint(x, y)
	s.AddConstraint(sketch.NewHorizontalDistance(s.Origin(), p, x), sketch.NewVerticalDistance(s.Origin(), p, y))
	return p
}

// bevelRevolved revolves a closed polygon given as (axial station, radius)
// about the world X axis.
func bevelRevolved(t *testing.T, doc *decad.Document, outline [][2]float64) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(outline))
	for i, xy := range outline {
		points[i] = bevelDimensioned(s, xy[0], xy[1])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

func (m bevelMember) endCone(t *testing.T, doc *decad.Document, apex float64) *decad.Body {
	t.Helper()
	far := -0.1 * m.coneDist
	return bevelRevolved(t, doc, [][2]float64{{far, 0}, {far, (apex - far) / math.Tan(m.coneAngle)}, {apex, 0}})
}

// toothProfile draws the tooth in its back-cone plane and returns the region
// that is the tooth.
func (m bevelMember) toothProfile(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(m.heelApex, 0, 0),
		r3.NewVec(math.Sin(m.coneAngle), -math.Cos(m.coneAngle), 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	n := len(m.flank)
	left, right := make([]*sketch.Point, n), make([]*sketch.Point, n)
	for i, xy := range m.flank {
		if i == n-1 {
			// The tip arc's radius relation fixes the radial coordinate.
			left[i] = s.CreatePoint(xy[0], xy[1])
			s.AddConstraint(sketch.NewVerticalDistance(s.Origin(), left[i], xy[1]))
		} else {
			left[i] = bevelDimensioned(s, xy[0], xy[1])
		}
		right[i] = bevelDimensioned(s, xy[0], -xy[1])
	}
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(s.Origin(), right[n-1], left[n-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	if m.embedded {
		root := s.CreateCircle(s.Origin(), m.rootRadius)
		s.AddConstraint(sketch.NewRadius(root, m.rootRadius))
	} else {
		scale := m.rootRadius / m.baseRoot
		rightRoot := bevelDimensioned(s, m.flank[0][0]*scale, -m.flank[0][1]*scale)
		leftRoot := s.CreatePoint(m.flank[0][0]*scale, m.flank[0][1]*scale)
		s.AddConstraint(sketch.NewVerticalDistance(s.Origin(), leftRoot, m.flank[0][1]*scale))
		s.CreateLine(rightRoot, right[0])
		s.CreateLine(left[0], leftRoot)
		s.CreateArc(s.Origin(), rightRoot, leftRoot)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	for _, candidate := range s.Profiles() {
		if !candidate.Valid || len(candidate.Holes) != 0 {
			continue
		}
		if !m.embedded {
			if len(candidate.Outer) == 6 {
				return s, candidate
			}
			continue
		}
		trimmed := 0
		for _, edge := range candidate.Outer {
			if _, ok := edge.Entity.(*sketch.FitSpline); ok && edge.Partial && edge.TExact {
				trimmed++
			}
		}
		if len(candidate.Outer) == 4 && trimmed == 2 {
			return s, candidate
		}
	}
	require.FailNow(t, "no tooth region")
	return nil, nil
}

// tooth lofts one tooth from the common apex and trims it at the toe cone and
// at the heel cone, the latter with opts.
func (m bevelMember) tooth(t *testing.T, doc *decad.Document, opts ...decad.BooleanOption) (*decad.Body, error) {
	t.Helper()
	s, profile := m.toothProfile(t)
	loft, err := decad.LoftFromPoint(t.Context(), doc, r3.Vec{}, s, profile)
	require.NoError(t, err)
	toeKept, err := decad.Cut(t.Context(), loft, m.endCone(t, doc, m.toeApex))
	require.NoError(t, err)
	return decad.Intersect(t.Context(), toeKept, m.endCone(t, doc, m.heelApex), opts...)
}

// join places count teeth around the blank and joins them in one Union.
func (m bevelMember) join(t *testing.T, count int, opts ...decad.BooleanOption) (*decad.Body, *decad.Body, error) {
	t.Helper()
	doc := decad.New()
	blank := bevelRevolved(t, doc, m.blank)
	tooth, err := m.tooth(t, doc, opts...)
	require.NoError(t, err)
	pattern := []*decad.Body{tooth}
	for i := 1; i < count; i++ {
		turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(2*math.Pi*float64(i)/float64(count)))
		require.NoError(t, err)
		copied, err := decad.Copy(t.Context(), tooth, decad.WithCopyPlacement(turn))
		require.NoError(t, err)
		pattern = append(pattern, copied)
	}
	joined, err := decad.Union(t.Context(), blank, pattern...)
	return blank, joined, err
}

func requireVerified(t *testing.T, body *decad.Body) {
	t.Helper()
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// requireTeethAddGain checks that count teeth add count times the volume one
// tooth adds, within the proven bounds.
func requireTeethAddGain(t *testing.T, m bevelMember, count int, blank, joined *decad.Body, opts ...decad.BooleanOption) {
	t.Helper()
	_, single, err := m.join(t, 1, opts...)
	require.NoError(t, err)
	blankVolume, err := blank.Volume()
	require.NoError(t, err)
	singleVolume, err := single.Volume()
	require.NoError(t, err)
	volume, err := joined.Volume()
	require.NoError(t, err)
	gain := singleVolume.Value.Base() - blankVolume.Value.Base()
	require.Greater(t, gain, 0.0)
	slack := volume.Bound.Base() + blankVolume.Bound.Base() +
		float64(count)*(singleVolume.Bound.Base()+blankVolume.Bound.Base())
	require.InDelta(t, float64(count)*gain, volume.Value.Base()-blankVolume.Value.Base(), slack)
}

// TestUnionBevelBlankAndEmbeddedRootTeeth joins a blank with three copies of
// a tooth whose root is a Circle fragment between two partial fit-spline
// flanks. The Union must verify, and the three teeth must add three times the
// gain of one tooth.
func TestUnionBevelBlankAndEmbeddedRootTeeth(t *testing.T) {
	if testing.Short() {
		t.Skip("joins three sectors")
	}
	blank, joined, err := bevel32.join(t, 3)
	require.NoError(t, err)
	requireVerified(t, joined)
	requireTeethAddGain(t, bevel32, 3, blank, joined)
}

// The 16-tooth member's tooth plane touches the heel cone along a ruling. Its
// tip nodes meet the cone to within rounding, and rounding leaves one of them
// a hair short, so the plain Intersect-and-Union is refused. The caller says
// the faces are meant to match and the join goes through.
func TestUnionBevelTipTouchingHeelConeNeedsMatchingFaces(t *testing.T) {
	if testing.Short() {
		t.Skip("joins two sectors")
	}
	blank, joined, err := bevel16.join(t, 2, decad.WithMatchingFaces())
	require.NoError(t, err)
	requireVerified(t, joined)
	requireTeethAddGain(t, bevel16, 2, blank, joined, decad.WithMatchingFaces())
}

func TestUnionBevelTipTouchingHeelConeRefusesWithoutMatchingFaces(t *testing.T) {
	if testing.Short() {
		t.Skip("joins two sectors")
	}
	_, _, err := bevel16.join(t, 2)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "far source plane")
}
