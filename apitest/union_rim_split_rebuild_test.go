package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// A strutted post: a round post leaning from a revolved plate, with revolved
// struts (a thin neck, a cone and a round strut) climbing at a steep angle into
// the post's axis, every strut on the same side of the post. The post's wall
// facets run its whole length, so each strut's rim lies on facets that the
// next strut and the plate also cross.
const (
	strutPostRadius     = 2.0
	strutPostLength     = 50.0
	strutPostFoot       = 0.6 // the post's foot height, inside the plate
	strutPlateRadius    = 15.0
	strutPlateThickness = 1.2
	strutNeckRadius     = 0.3
	strutNeckLength     = 1.4
	strutTaper          = 3.0
	strutRadius         = 0.8
	strutLength         = 10.0
	strutFirst          = 14.0 // the lowest strut's end, along the post axis from its foot
	strutPitch          = 6.0  // the spacing of the strut ends along the post axis
)

var (
	strutPostLean = 9 * math.Pi / 180
	strutRise     = 60 * math.Pi / 180
	strutApproach = 30 * math.Pi / 180
)

// strutPart is one part of the strutted post: the plate, the post or a strut.
type strutPart int

const (
	strutPlate strutPart = -2
	strutPost  strutPart = -1
)

// strutBuilder builds the parts in one document, reflected in x = 0 when
// mirrored.
type strutBuilder struct {
	t      *testing.T
	doc    *decad.Document
	mirror bool
}

func (b strutBuilder) place(v r3.Vec) r3.Vec {
	if b.mirror {
		v.X = -v.X
	}
	return v
}

func (b strutBuilder) frame(origin, u, v r3.Vec) *sketch.Sketch {
	b.t.Helper()
	f, err := r3.NewFrame(origin, u, v)
	require.NoError(b.t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(b.t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(b.t, err)
	return s
}

// revolve revolves the closed polygon pts, drawn on the frame (origin, u, v),
// a full turn about the line through origin along u.
func (b strutBuilder) revolve(origin, u, v r3.Vec, pts [][2]float64) *decad.Body {
	b.t.Helper()
	s := b.frame(origin, u, v)
	ps := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		ps[i] = s.CreatePoint(p[0], p[1])
		s.Fix(ps[i])
	}
	for i := range ps {
		s.CreateLine(ps[i], ps[(i+1)%len(ps)])
	}
	_, err := s.Solve(b.t.Context())
	require.NoError(b.t, err)
	require.Len(b.t, s.Profiles(), 1)
	body, err := decad.Revolve(b.t.Context(), b.doc, s, s.Profiles()[0],
		decad.ConstructionAxis{Origin: origin, Dir: u}, decad.FullRevolution{})
	require.NoError(b.t, err)
	return body
}

func (b strutBuilder) postAxis() r3.Vec {
	return b.place(r3.NewVec(math.Sin(strutPostLean), 0, math.Cos(strutPostLean)))
}

func (b strutBuilder) postFoot() r3.Vec { return r3.NewVec(0, 0, strutPostFoot) }

func (b strutBuilder) part(p strutPart) *decad.Body {
	b.t.Helper()
	switch p {
	case strutPlate:
		return b.revolve(r3.Vec{}, r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), [][2]float64{
			{0, 0}, {strutPlateThickness, 0}, {strutPlateThickness, strutPlateRadius}, {0, strutPlateRadius},
		})
	case strutPost:
		axis := b.postAxis()
		u, ok := r3.NewVec(0, 0, 1).Cross(axis).Normalize()
		require.True(b.t, ok)
		s := b.frame(b.postFoot(), u, axis.Cross(u))
		s.AddConstraint(sketch.NewRadius(s.CreateCircle(s.Origin(), strutPostRadius), strutPostRadius))
		_, err := s.Solve(b.t.Context())
		require.NoError(b.t, err)
		require.Len(b.t, s.Profiles(), 1)
		body, err := decad.Extrude(b.t.Context(), b.doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(strutPostLength), Dir: decad.Along})
		require.NoError(b.t, err)
		return body
	}
	end := b.postFoot().Add(b.postAxis().Scale(strutFirst + strutPitch*float64(p)))
	in := b.place(r3.NewVec(-math.Cos(strutApproach), -math.Sin(strutApproach), 0))
	w := in.Scale(math.Cos(strutRise)).Add(r3.NewVec(0, 0, math.Sin(strutRise)))
	start := end.Sub(w.Scale(strutLength))
	v, ok := r3.NewVec(0, 0, 1).Cross(w).Normalize()
	require.True(b.t, ok)
	return b.revolve(start, w, v, [][2]float64{
		{0, 0}, {0, strutNeckRadius}, {strutNeckLength, strutNeckRadius},
		{strutNeckLength + strutTaper, strutRadius}, {strutLength, strutRadius}, {strutLength, 0},
	})
}

// strutOutside integrates, independently of the evaluator, the volume of one
// strut that lies outside the post, on a midpoint grid of nu steps along the
// strut and n by n across it. Every strut is the first one moved along the post
// axis, so each has the same volume outside the post.
func strutOutside(nu, n int) float64 {
	b := strutBuilder{}
	axis, foot := b.postAxis(), b.postFoot()
	end := foot.Add(axis.Scale(strutFirst))
	in := r3.NewVec(-math.Cos(strutApproach), -math.Sin(strutApproach), 0)
	w := in.Scale(math.Cos(strutRise)).Add(r3.NewVec(0, 0, math.Sin(strutRise)))
	start := end.Sub(w.Scale(strutLength))
	e1, _ := r3.NewVec(0, 0, 1).Cross(w).Normalize()
	e2 := w.Cross(e1)
	du, dp := strutLength/float64(nu), 2*strutRadius/float64(n)
	total := 0
	for i := range nu {
		u := (float64(i) + 0.5) * du
		rho := strutRadius
		switch {
		case u < strutNeckLength:
			rho = strutNeckRadius
		case u < strutNeckLength+strutTaper:
			rho = strutNeckRadius + (strutRadius-strutNeckRadius)*(u-strutNeckLength)/strutTaper
		}
		c := start.Add(w.Scale(u))
		for j := range n {
			p := -strutRadius + (float64(j)+0.5)*dp
			for k := range n {
				q := -strutRadius + (float64(k)+0.5)*dp
				if p*p+q*q > rho*rho {
					continue
				}
				d := c.Add(e1.Scale(p)).Add(e2.Scale(q)).Sub(foot)
				along := d.Dot(axis)
				if d.Dot(d)-along*along > strutPostRadius*strutPostRadius {
					total++
				}
			}
		}
	}
	return float64(total) * du * dp * dp
}

// strutChainTolerance is the strutted post's chain tolerance: above the bound
// one strut's rim leaves on the post wall, below the bound the rims of six
// struts reach when each is composed from the previous one's.
const (
	strutChainTolerance = 0.01
	strutCount          = 6
)

// strutOrders join the parts two at a time: each strut onto the post from the
// top down or from the bottom up and then the post onto the plate, or the post
// onto the plate first and then each strut from the bottom up.
var strutOrders = []struct {
	name string
	join func(t *testing.T, b strutBuilder) *decad.Body
}{
	{"top down", func(t *testing.T, b strutBuilder) *decad.Body {
		return b.joinPostFirst(t, func(j int) int { return strutCount - 1 - j })
	}},
	{"bottom up", func(t *testing.T, b strutBuilder) *decad.Body {
		return b.joinPostFirst(t, func(j int) int { return j })
	}},
	{"plate first", func(t *testing.T, b strutBuilder) *decad.Body {
		body, err := decad.Union(t.Context(), b.part(strutPlate), b.part(strutPost))
		require.NoError(t, err)
		for i := range strutCount {
			body, err = decad.Union(t.Context(), body, b.part(strutPart(i)))
			require.NoError(t, err, "strut %d", i)
		}
		return body
	}},
}

// joinPostFirst joins the struts onto the post in the order strut(0), strut(1),
// ..., then the post onto the plate.
func (b strutBuilder) joinPostFirst(t *testing.T, strut func(int) int) *decad.Body {
	t.Helper()
	post := b.part(strutPost)
	var err error
	for j := range strutCount {
		post, err = decad.Union(t.Context(), post, b.part(strutPart(strut(j))))
		require.NoError(t, err, "strut %d", strut(j))
	}
	body, err := decad.Union(t.Context(), b.part(strutPlate), post)
	require.NoError(t, err)
	return body
}

// TestUnionChainRimSplitRebuild joins a strutted post two Unions at a time in
// three orders, right-handed and mirrored. Each strut's rim lands on post wall
// facets that run on to the next strut and to the plate, so the sequential
// chain composes each rim from the last and the local chain-depth gate refuses
// it; the refused Union rebuilds the recorded Union history with rim splitting
// on and joins. Every order builds a watertight solid whose proven volume
// encloses the closed-form one.
func TestUnionChainRimSplitRebuild(t *testing.T) {
	t.Parallel()
	// The plate and post overlap where the post's centre line runs from its foot
	// to the plate's top face: the oblique cut leaves a cylinder of that length.
	plateAndPost := math.Pi*strutPlateRadius*strutPlateRadius*strutPlateThickness +
		math.Pi*strutPostRadius*strutPostRadius*strutPostLength -
		math.Pi*strutPostRadius*strutPostRadius*(strutPlateThickness-strutPostFoot)/math.Cos(strutPostLean)
	// The grid integral converges to within 0.002 mm^3 of the next finer grid.
	const gridAllowance = strutCount * 0.01
	want := plateAndPost + strutCount*strutOutside(1000, 160)
	for _, mirror := range []bool{false, true} {
		for _, order := range strutOrders {
			t.Run(fmt.Sprintf("%s mirrored=%v", order.name, mirror), func(t *testing.T) {
				t.Parallel()
				doc := decad.New(decad.WithChainTolerance(units.Millimeters(strutChainTolerance)))
				body := order.join(t, strutBuilder{t: t, doc: doc, mirror: mirror})
				require.Equal(t, []*decad.Body{body}, doc.Bodies())
				require.True(t, body.IsSolid())
				requireBodyWatertight(t, body)
				v, err := body.Volume()
				require.NoError(t, err)
				bound, err := v.Bound.In(units.CubicMillimeter)
				require.NoError(t, err)
				require.InDelta(t, want, volumeMM(t, v), bound+gridAllowance)
				require.Less(t, bound, 1e-3*want, "the rebuilt chain states a usable volume bound")
			})
		}
	}
}
