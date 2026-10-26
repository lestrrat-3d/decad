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

// Lines are the feature edges split by visibility.
type Lines struct {
	Visible, Hidden []Segment
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
		if isFeature(owners, normals, proj) {
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
		splitEdge(verts[e[0]], verts[e[1]], proj, depths, cam, &lines)
	}
	return lines, nil
}

func isFeature(owners []int, normals []r3.Vec, proj raster.Projection) bool {
	if len(owners) != 2 {
		return true
	}
	a, b := normals[owners[0]], normals[owners[1]]
	if a.Dot(b) < CreaseCosine {
		return true
	}
	return proj.Facing(a)*proj.Facing(b) < 0
}

// splitEdge cuts the edge from p to q into visible and hidden runs and appends
// them to lines.
func splitEdge(p, q r3.Vec, proj raster.Projection, depths []float64, cam raster.Camera, lines *Lines) {
	px, py, _ := proj.Project(p)
	qx, qy, _ := proj.Project(q)
	samples := min(max(int(math.Ceil(math.Hypot(qx-px, qy-py))), 1), 8192)
	base := 2 * proj.PixelSize()
	visibleAt := func(t float64) bool {
		x, y, depth := proj.Project(p.Add(q.Sub(p).Scale(t)))
		near, slope := nearest(depths, proj, x, y)
		return depth <= near+base+2*slope
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
		seg := Segment{A: plane(p.Add(q.Sub(p).Scale(t0)), cam), B: plane(p.Add(q.Sub(p).Scale(t1)), cam)}
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

// plane returns the drawing-plane coordinates of v under cam: along the view's
// right and up axes.
func plane(v r3.Vec, cam raster.Camera) [2]float64 {
	right, up, _ := cam.Axes()
	return [2]float64{right.Dot(v), up.Dot(v)}
}
