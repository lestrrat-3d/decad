// Package drawing projects a triangle mesh's feature edges onto a plane and
// splits them into visible and hidden runs (agent-docs/dxf-export-design.md).
// It works on plain millimetre data and touches no body or document.
package drawing

import (
	"context"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/raster"
	"github.com/lestrrat-3d/r3"
)

// CreaseCosine is the cosine of the dihedral turn above which an edge shared
// by two triangles is a feature: a turn of more than 30 degrees.
var CreaseCosine = math.Cos(30 * math.Pi / 180)

// visibilityPixels is the side, in pixels, of the depth buffer that decides
// which part of an edge is hidden.
const visibilityPixels = 2048

// Segment is a straight piece of an edge in the drawing plane, in
// millimetres. X runs to the right of the view and Y up it, measured from the
// world origin's projection.
type Segment struct {
	A, B [2]float64
}

// Circle is a whole circle in the drawing plane, in millimetres, as Segment's
// axes measure it.
type Circle struct {
	Centre [2]float64
	Radius float64
}

// Arc is a circular arc in the drawing plane, in millimetres, counter-clockwise
// seen in the drawing from Start to End, both angles in radians from the X axis.
// End is Start plus the sweep, so it may pass 2π.
type Arc struct {
	Centre     [2]float64
	Radius     float64
	Start, End float64
}

// Ends returns the arc's two end points.
func (a Arc) Ends() (start, end [2]float64) {
	at := func(angle float64) [2]float64 {
		return [2]float64{a.Centre[0] + a.Radius*math.Cos(angle), a.Centre[1] + a.Radius*math.Sin(angle)}
	}
	return at(a.Start), at(a.End)
}

// Bounds returns the box of the arc: its end points and each axis extreme of the
// circle the arc passes through.
func (a Arc) Bounds() (minX, maxX, minY, maxY float64) {
	s, e := a.Ends()
	minX, maxX = math.Min(s[0], e[0]), math.Max(s[0], e[0])
	minY, maxY = math.Min(s[1], e[1]), math.Max(s[1], e[1])
	for k := range 4 {
		angle := float64(k) * math.Pi / 2
		// The extreme at angle is on the arc when it lies past Start within the sweep.
		if math.Mod(angle-a.Start+4*math.Pi, 2*math.Pi) <= a.End-a.Start {
			x, y := a.Centre[0]+a.Radius*math.Cos(angle), a.Centre[1]+a.Radius*math.Sin(angle)
			minX, maxX = math.Min(minX, x), math.Max(maxX, x)
			minY, maxY = math.Min(minY, y), math.Max(maxY, y)
		}
	}
	return minX, maxX, minY, maxY
}

// Lines are the feature edges split by visibility. VisibleCircles, HiddenCircles,
// VisibleArcs and HiddenArcs are circles and arcs that Circulate and
// CirculateArcs took out of Visible and Hidden.
type Lines struct {
	Visible, Hidden               []Segment
	VisibleCircles, HiddenCircles []Circle
	VisibleArcs, HiddenArcs       []Arc
}

// circleOnTolerance is how far, relative to a circle's radius (at least 1 mm), a
// segment end may lie from the circle and still be one of its chords.
const circleOnTolerance = 1e-7

// Circulate replaces the segments that make up each candidate circle with the
// circle itself. A circle is taken when the visible segments lying on it, both
// ends on the circle, together go all the way round it, so that every one of its
// chords is a whole visible segment; then those segments are removed and the
// circle joins VisibleCircles. Otherwise the same test is made against the hidden
// segments. A circle some of whose chords are cut by a surface, or whose chords
// are not all there, is left as the segments it is. A segment lying on a taken
// circle is removed from its list only; a candidate that coincides with another
// circle's segments takes its own.
func (l *Lines) Circulate(candidates []Circle) {
	for _, c := range candidates {
		if rest, ok := coverCircle(l.Visible, c); ok {
			l.Visible = rest
			l.VisibleCircles = append(l.VisibleCircles, c)
		}
		if rest, ok := coverCircle(l.Hidden, c); ok {
			l.Hidden = rest
			l.HiddenCircles = append(l.HiddenCircles, c)
		}
	}
}

// CirculateArcs is Circulate for arcs: an arc is taken when the visible segments
// with both ends on its circle and inside its sweep together run all the way along
// it, each a whole chord, and then those segments are removed and the arc joins
// VisibleArcs; otherwise the hidden segments are tried for HiddenArcs.
func (l *Lines) CirculateArcs(candidates []Arc) {
	for _, a := range candidates {
		if rest, ok := coverArc(l.Visible, a); ok {
			l.Visible = rest
			l.VisibleArcs = append(l.VisibleArcs, a)
		}
		if rest, ok := coverArc(l.Hidden, a); ok {
			l.Hidden = rest
			l.HiddenArcs = append(l.HiddenArcs, a)
		}
	}
}

// coverArc splits segments into the chords of arc a and the rest, and reports
// whether the chords run the whole sweep.
func coverArc(segments []Segment, a Arc) (rest []Segment, covered bool) {
	eps := circleOnTolerance * math.Max(1, a.Radius)
	sweep := a.End - a.Start
	// into is how far past Start, counter-clockwise, the point lies on the circle, or false
	// when the point is off the circle or outside the sweep. A point just before Start
	// counts as zero.
	into := func(p [2]float64) (float64, bool) {
		if math.Abs(math.Hypot(p[0]-a.Centre[0], p[1]-a.Centre[1])-a.Radius) > eps {
			return 0, false
		}
		d := math.Mod(math.Atan2(p[1]-a.Centre[1], p[0]-a.Centre[0])-a.Start+4*math.Pi, 2*math.Pi)
		if d > 2*math.Pi-1e-9 {
			d = 0
		}
		return d, d <= sweep+1e-9
	}
	mine := make([]bool, len(segments))
	span := 0.0
	for i, s := range segments {
		da, okA := into(s.A)
		db, okB := into(s.B)
		if !okA || !okB {
			continue
		}
		mine[i] = true
		span += math.Abs(db - da)
	}
	if span < sweep-1e-6 {
		return segments, false
	}
	for i, s := range segments {
		if !mine[i] {
			rest = append(rest, s)
		}
	}
	return rest, true
}

// coverCircle splits segments into those on circle c and the rest, and reports
// whether those on c go all the way round it.
func coverCircle(segments []Segment, c Circle) (rest []Segment, covered bool) {
	eps := circleOnTolerance * math.Max(1, c.Radius)
	on := func(p [2]float64) bool {
		return math.Abs(math.Hypot(p[0]-c.Centre[0], p[1]-c.Centre[1])-c.Radius) <= eps
	}
	span := 0.0
	mine := make([]bool, len(segments))
	for i, s := range segments {
		if !on(s.A) || !on(s.B) {
			continue
		}
		mine[i] = true
		a := math.Atan2(s.A[1]-c.Centre[1], s.A[0]-c.Centre[0])
		b := math.Atan2(s.B[1]-c.Centre[1], s.B[0]-c.Centre[0])
		d := math.Mod(b-a+3*math.Pi, 2*math.Pi) - math.Pi
		span += math.Abs(d)
	}
	if span < 2*math.Pi-1e-6 {
		return segments, false
	}
	for i, s := range segments {
		if !mine[i] {
			rest = append(rest, s)
		}
	}
	return rest, true
}

type edgeKey [2]int

func keyOf(a, b int) edgeKey {
	if a > b {
		a, b = b, a
	}
	return edgeKey{a, b}
}

// Project finds the feature edges of the mesh and projects them under cam. An
// edge is a feature when exactly one triangle uses it or three or more do, when
// its two triangles turn by more than CreaseCosine, or when one of them looks
// toward the camera and the other away from it (a silhouette). Every other
// edge, such as the diagonal of a flat face, is left out.
//
// An edge is cut into runs by sampling it about once per pixel of a 2048-pixel
// depth buffer: a sample is hidden when a triangle in front of it covers its
// pixel or a neighbouring one. The result is exact to a pixel of that buffer, so
// an edge that passes behind a surface by less than two pixels of its size plus
// twice the surface's depth change per pixel reads as visible. The segments are ordered by edge, then along the edge, so the
// result does not depend on map order.
func Project(ctx context.Context, verts []r3.Vec, tris [][3]int, cam raster.Camera) (Lines, error) {
	proj, err := raster.NewProjection(verts, cam, visibilityPixels, visibilityPixels)
	if err != nil {
		return Lines{}, err
	}
	depths, err := raster.DepthBuffer(ctx, verts, tris, proj)
	if err != nil {
		return Lines{}, err
	}
	normals := make([]r3.Vec, len(tris))
	for i, t := range tris {
		n, _ := verts[t[1]].Sub(verts[t[0]]).Cross(verts[t[2]].Sub(verts[t[0]])).Normalize()
		normals[i] = n // the zero vector for a triangle of no area
	}
	centres := make([]r3.Vec, len(tris))
	for i, t := range tris {
		centres[i] = verts[t[0]].Add(verts[t[1]]).Add(verts[t[2]]).Scale(1.0 / 3)
	}
	incident := make(map[edgeKey][]int, 3*len(tris))
	for ti, t := range tris {
		if normals[ti] == (r3.Vec{}) {
			continue
		}
		for k := range t {
			e := keyOf(t[k], t[(k+1)%3])
			incident[e] = append(incident[e], ti)
		}
	}
	edges := make([]edgeKey, 0, len(incident))
	for e, owners := range incident {
		if isFeature(owners, normals, centres, proj) {
			edges = append(edges, e)
		}
	}
	slices.SortFunc(edges, func(a, b edgeKey) int {
		if a[0] != b[0] {
			return a[0] - b[0]
		}
		return a[1] - b[1]
	})
	var lines Lines
	for i, e := range edges {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return Lines{}, err
			}
		}
		splitEdge(verts[e[0]], verts[e[1]], proj, depths, &lines)
	}
	return lines, nil
}

func isFeature(owners []int, normals, centres []r3.Vec, proj raster.Projection) bool {
	if len(owners) != 2 {
		return true
	}
	a, b := normals[owners[0]], normals[owners[1]]
	if a.Dot(b) < CreaseCosine {
		return true
	}
	return proj.FacingAt(a, centres[owners[0]])*proj.FacingAt(b, centres[owners[1]]) < 0
}

// splitEdge cuts the edge from p to q into visible and hidden runs and appends
// them to lines.
func splitEdge(p, q r3.Vec, proj raster.Projection, depths []float64, lines *Lines) {
	px, py, _ := proj.Project(p)
	qx, qy, _ := proj.Project(q)
	samples := min(max(int(math.Ceil(math.Hypot(qx-px, qy-py))), 1), 8192)
	visibleAt := func(t float64) bool {
		x, y, depth := proj.Project(p.Add(q.Sub(p).Scale(t)))
		near, slope := nearest(depths, proj, x, y)
		return depth <= near+proj.DepthTolerance(depth)+2*slope
	}
	runs := []run{{first: 0, visible: visibleAt(0.5 / float64(samples))}}
	for k := 1; k < samples; k++ {
		v := visibleAt((float64(k) + 0.5) / float64(samples))
		if v == runs[len(runs)-1].visible {
			continue
		}
		runs[len(runs)-1].end = k
		runs = append(runs, run{first: k, visible: v})
	}
	runs[len(runs)-1].end = samples
	runs = absorbShortRuns(runs)
	for _, r := range runs {
		t0, t1 := float64(r.first)/float64(samples), float64(r.end)/float64(samples)
		seg := Segment{A: plane(p.Add(q.Sub(p).Scale(t0)), proj), B: plane(p.Add(q.Sub(p).Scale(t1)), proj)}
		if r.visible {
			lines.Visible = append(lines.Visible, seg)
		} else {
			lines.Hidden = append(lines.Hidden, seg)
		}
	}
}

// run is the samples [first, end) of an edge that share one visibility.
type run struct {
	first, end int
	visible    bool
}

// shortRun is the number of samples below which a run of one visibility between
// two of the other, or at an end beside one, is the buffer's pixel noise at a
// corner and not a real change.
const shortRun = 3

// absorbShortRuns merges each run shorter than shortRun into its neighbour, the
// longer one where it has two, and joins the neighbours that then agree. A lone
// run stays as it is.
func absorbShortRuns(runs []run) []run {
	for len(runs) > 1 {
		shortest := -1
		for i, r := range runs {
			if r.end-r.first < shortRun && (shortest < 0 || r.end-r.first < runs[shortest].end-runs[shortest].first) {
				shortest = i
			}
		}
		if shortest < 0 {
			break
		}
		r := runs[shortest]
		target := shortest - 1
		if shortest == 0 || (shortest+1 < len(runs) && runs[shortest+1].end-runs[shortest+1].first > runs[target].end-runs[target].first) {
			target = shortest + 1
		}
		runs[target].first = min(runs[target].first, r.first)
		runs[target].end = max(runs[target].end, r.end)
		runs = slices.Delete(runs, shortest, shortest+1)
		// Neighbours of one visibility join.
		for i := 1; i < len(runs); i++ {
			if runs[i].visible == runs[i-1].visible {
				runs[i-1].end = runs[i].end
				runs = slices.Delete(runs, i, i+1)
				i--
			}
		}
	}
	return runs
}

// nearest is the smallest depth in the 3 x 3 pixels around (x, y), and how
// much the depth changes between that pixel and the covered pixels around it,
// which is how far a point on that surface can lie from the buffer's value at
// its own pixel. Both are +Inf and 0 outside the buffer or over bare background.
func nearest(depths []float64, proj raster.Projection, x, y float64) (depth, slope float64) {
	at := func(ix, iy int) float64 {
		if ix < 0 || iy < 0 || ix >= proj.Width || iy >= proj.Height {
			return math.Inf(1)
		}
		return depths[iy*proj.Width+ix]
	}
	cx, cy := int(math.Floor(x)), int(math.Floor(y))
	depth = math.Inf(1)
	bx, by := cx, cy
	for dy := -1; dy <= 1; dy++ {
		for dx := -1; dx <= 1; dx++ {
			if d := at(cx+dx, cy+dy); d < depth {
				depth, bx, by = d, cx+dx, cy+dy
			}
		}
	}
	for dy := -1; dy <= 1; dy++ {
		for dx := -1; dx <= 1; dx++ {
			if d := at(bx+dx, by+dy); !math.IsInf(d, 1) {
				slope = math.Max(slope, math.Abs(d-depth))
			}
		}
	}
	return depth, slope
}

// plane returns the drawing-plane coordinates of v under proj: along the view's
// right and up axes, or over the distance from the eye for a perspective one.
func plane(v r3.Vec, proj raster.Projection) [2]float64 {
	x, y := proj.PlanePoint(v)
	return [2]float64{x, y}
}
