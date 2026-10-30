package retainedblend

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// rewriteCylCyl blends one cylinder-cylinder line (R5) and builds the body
// over the rewritten mesh. Both walls are cut at their setback generators as
// the wall of R3 is; the strip joins the two cuts.
func rewriteCylCyl(ctx context.Context, env topo.Env, fp facetedrecord.Payload, c cylCyl, spec blendSpec,
	role string, ord int, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	w := newWork(fp, rim{scale: c.scale, charge: c.charge}, spec)
	slack := c.slack()
	margin := w.tau + slack
	k0, k1 := c.ends[0].exact, c.ends[1].exact
	axis := c.axis
	length := k1.Sub(k0).Dot(axis)
	if !(length > 4*margin) {
		return nil, fmt.Errorf("%w: the edge is too short for the blend", decaderr.ErrDegenerate)
	}

	var faces [4]faceLoops
	for k, f := range []*topo.Face{c.wall[0].face, c.wall[1].face, c.ends[0].face, c.ends[1].face} {
		var err error
		var end *plane
		if k >= 2 {
			end = &c.ends[k-2].g
		}
		if faces[k], err = w.faceLoops(faceIndex[f], end); err != nil {
			return nil, err
		}
	}
	fG := [2]faceLoops{faces[2], faces[3]}

	// Locate the edge in each wall's loops.
	onEdge := func(v int) bool { return distToLine(w.verts[v], k0, axis) <= w.beta[v]+slack }
	var runs [2]edgeRun
	for k := range 2 {
		var onArc [2]func(int) bool
		for i := range 2 {
			oi := c.wall[k].axisPoint(axis, c.ends[i].exact)
			radius := c.wall[k].radius
			onArc[i] = func(v int) bool {
				q := w.verts[v].Sub(oi)
				axial := q.Dot(axis)
				radial := q.Sub(axis.Scale(axial)).Len()
				return math.Hypot(radial-radius, axial) <= w.beta[v]+slack
			}
		}
		var err error
		if runs[k], err = w.locateBy(faces[k], c.ends[0].v, c.ends[1].v, onEdge, onArc); err != nil {
			return nil, err
		}
	}

	// The corner's frame: radial units, ways into the faces, wall normals.
	var o, u, wt, nrm [2]r3.Vec
	for k := range 2 {
		o[k] = c.wall[k].axisPoint(axis, k0)
		u[k] = k0.Sub(o[k]).Scale(1 / c.wall[k].radius)
		nrm[k] = u[k]
		if c.wall[k].matOutside {
			nrm[k] = u[k].Scale(-1)
		}
		tangent := axis.Cross(u[k])
		toward := w.verts[runs[k].chain[0][0]].Sub(w.verts[c.ends[0].v])
		if !clears(math.Abs(toward.Dot(tangent)), toward.Len()) {
			return nil, fmt.Errorf("%w: a wall leaves the edge end along no clear direction", decaderr.ErrUnsupported)
		}
		wt[k] = tangent
		if toward.Dot(tangent) < 0 {
			wt[k] = tangent.Scale(-1)
		}
		toward1 := w.verts[runs[k].chain[1][0]].Sub(w.verts[c.ends[1].v])
		if !clears(toward1.Dot(wt[k]), toward1.Len()) {
			return nil, fmt.Errorf("%w: the wall turns reflex at an edge end", decaderr.ErrUnsupported)
		}
	}
	if shallow(axis.Cross(u[0]).Cross(axis.Cross(u[1])).Len()) {
		return nil, fmt.Errorf("%w: the two walls meet at a grazing angle", decaderr.ErrUnsupported)
	}
	convex := nrm[1].Dot(wt[0]) < 0
	if (nrm[0].Dot(wt[1]) < 0) != convex || c.edgeConvex != convex {
		return nil, fmt.Errorf("%w: the edge's convexity disagrees with its two tags", decaderr.ErrUnsupported)
	}

	sec, err := c.solveSection(spec, k0, o, u, wt, convex, w.tau, margin)
	if err != nil {
		return nil, err
	}
	var cuts [2]wallCut
	for k := range 2 {
		cuts[k], err = w.cutWall(fp, faces[k], runs[k], c, c.wall[k], o[k], u[k], wt[k], sec.theta[k], k0)
		if err != nil {
			return nil, err
		}
	}

	// The rows: wall 0's cut, the blend's interior rows at that cut's places on
	// the axis, wall 1's cut.
	m := len(sec.rows) - 1
	rows := make([][]int, m+1)
	rows[0], rows[m] = cuts[0].cols, cuts[1].cols
	bound := w.tau + slack
	for j := 1; j < m; j++ {
		for k, id := range cuts[0].cols {
			sigma := w.verts[id].Sub(k0).Dot(axis)
			if k == 0 {
				sigma = 0
			} else if k == len(cuts[0].cols)-1 {
				sigma = length
			}
			rows[j] = append(rows[j], w.vertex(sec.rows[j].Add(axis.Scale(sigma)), bound))
		}
	}

	// The end faces.
	var gLoops [2][][]int
	for i := range 2 {
		pick := func(r []int) int {
			if i == 0 {
				return r[0]
			}
			return r[len(r)-1]
		}
		end := make([]int, m+1)
		reach := make([]r3.Vec, m+1)
		for j := range end {
			end[j] = pick(rows[j])
			reach[j] = w.verts[end[j]]
		}
		kinds := [2]sideKind{trims, trims}
		loops, _, err := w.endFace(c.ends[i], fG[i], runs[0].chain[i], runs[1].chain[i], kinds,
			cuts[0].rem[i], cuts[1].rem[i], end, reach, margin, convex, nil)
		if err != nil {
			return nil, err
		}
		gLoops[i] = loops
	}
	for i, f := range []struct {
		n, o r3.Vec
	}{{c.ends[0].g.n, k0}, {c.ends[1].g.n, k1}} {
		if err := w.retriangulate(f.n, f.o, gLoops[i], fG[i].group); err != nil {
			return nil, err
		}
		w.removed = append(w.removed, fG[i].facets...)
	}

	// The strip, and its tag.
	tag := c.tag(spec, sec, nrm, convex)
	blendGroup := len(fp.Groups)
	stripFrom := len(w.out)
	for j := range m {
		for _, t := range w.zipOpen(rows[j], rows[j+1], k0, axis) {
			va, vb, vc := w.verts[t[0]], w.verts[t[1]], w.verts[t[2]]
			centroid := va.Add(vb).Add(vc).Scale(1.0 / 3)
			if vb.Sub(va).Cross(vc.Sub(va)).Dot(tag.outward(centroid)) < 0 {
				t[1], t[2] = t[2], t[1]
			}
			w.emit(t, blendGroup)
		}
	}
	corner := c.cornerArea(spec, sec, k0, o, wt)
	if !convex {
		corner = -corner
	}
	w.removedTrue = length*(c.wall[0].radius*sec.theta[0]+c.wall[1].radius*sec.theta[1]) + 2*corner
	if spec.fillet {
		tag.trueArea = spec.r * sec.phi * length
	} else {
		tag.trueArea = sec.t[1].Sub(sec.t[0]).Len() * length
	}

	groups := append([]facetedrecord.Group(nil), fp.Groups...)
	groups = append(groups, w.tagGroup(facetedrecord.Group{
		Origins: []provenance.Ref{{Producer: env.Ref, Role: role}},
		Source:  facetedrecord.BlendSource(env.Ref, ord),
		Planar:  !spec.fillet,
	}, tag))
	for _, f := range faces {
		groups[f.group].HasCentroid = false
		groups[f.group].Centroid = measurement.VecMeasurement{}
	}
	return w.buildClear(ctx, env, groups, stripFrom, tag, faceIndex)
}

// tag is the exact surface of the blend face.
func (c cylCyl) tag(spec blendSpec, sec cylSection, nrm [2]r3.Vec, convex bool) blendTag {
	if spec.fillet {
		tag := blendTag{
			surface:  surfacegeom.Cylinder{Origin: sec.centre, Axis: c.axis, Radius: units.Millimeters(spec.r)},
			reversed: !convex,
		}
		tag.outward = func(p r3.Vec) r3.Vec {
			q := p.Sub(sec.centre)
			q = q.Sub(c.axis.Scale(q.Dot(c.axis)))
			if !convex {
				q = q.Scale(-1)
			}
			return q
		}
		return tag
	}
	chord, _ := sec.t[1].Sub(sec.t[0]).Normalize()
	frame, err := r3.NewFrame(sec.t[0], c.axis, chord)
	// The chamfer faces away from the material: toward the bisector of the
	// walls' outward normals at the corner.
	n := frame.N()
	away := nrm[0].Add(nrm[1])
	tag := blendTag{surface: surfacegeom.Plane{Frame: frame}}
	if err == nil {
		tag.reversed = n.Dot(away) < 0
	}
	if n.Dot(away) < 0 {
		n = n.Scale(-1)
	}
	tag.outward = func(r3.Vec) r3.Vec { return n }
	return tag
}
