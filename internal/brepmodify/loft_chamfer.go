package brepmodify

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
	"github.com/lestrrat-3d/r3"
)

// LoftCapChamfer is the matching axial loft's complete outer-cap loop
// route. Its cap contour follows the loft's certified held station polygon.
func LoftCapChamfer(ctx context.Context, env topo.Env, b *topo.Body, pl loftrecord.Payload, edges []*topo.Edge,
	d, dDelta float64, asym *Asymmetric) (*topo.Body, bool, error) {
	if asym != nil || pl.SurfaceResult || pl.Xform != r3.Identity() ||
		!sectionrecord.IdenticalRecord(pl.Profile0, pl.Profile1) ||
		len(pl.Profile0.Holes) > 1 ||
		pl.Frame0.U() != pl.Frame1.U() || pl.Frame0.V() != pl.Frame1.V() {
		return nil, false, nil
	}
	if len(pl.Profile0.Holes) == 1 {
		if len(pl.Profile0.Holes[0].Segments) != 1 {
			return nil, false, nil
		}
		if _, ok := pl.Profile0.Holes[0].Segments[0].(sectionrecord.CircleSeg); !ok {
			return nil, false, nil
		}
	}
	for _, offset := range pl.Alignment {
		if offset != 0 {
			return nil, false, nil
		}
	}
	end := pl.Frame0.ToLocal(pl.Frame1.Origin())
	if end.X != 0 || end.Y != 0 || end.Z <= 2*d {
		return nil, false, nil
	}
	var caps [2]*topo.Face
	for _, face := range topobuild.Faces(b) {
		for _, origin := range face.Origins {
			if origin == capRef(b, provenance.RoleCapStart) {
				caps[0] = face
			}
			if origin == capRef(b, provenance.RoleCapEnd) {
				caps[1] = face
			}
		}
	}
	if caps[0] == nil || caps[1] == nil {
		return nil, false, nil
	}
	if !loftcontact.MatchingHeldCapOuters(pl.Frame0, caps[0], caps[1]) {
		return nil, false, nil
	}
	selected := make(map[*topo.Edge]struct{}, len(edges))
	for _, edge := range edges {
		selected[edge] = struct{}{}
	}
	for _, endCap := range caps {
		if len(endCap.Loops) == 0 || !endCap.Loops[0].Outer {
			return nil, false, nil
		}
		for _, ce := range endCap.Loops[0].CoEdges {
			edge := ce.Edge
			if _, ok := selected[edge]; !ok {
				return nil, false, nil
			}
			delete(selected, edge)
		}
	}
	if len(selected) != 0 {
		return nil, false, nil
	}
	outer := make([]sectionrecord.CurveSegment, 0, len(caps[1].Loops[0].CoEdges))
	for _, ce := range caps[1].Loops[0].CoEdges {
		start := pl.Frame0.ToLocal(topobuild.CoEdgeStart(ce).Position)
		stop := pl.Frame0.ToLocal(topobuild.CoEdgeEnd(ce).Position)
		outer = append(outer, sectionrecord.LineSeg{
			Start:  sectionrecord.Point2{U: start.X, V: start.Y},
			End:    sectionrecord.Point2{U: stop.X, V: stop.Y},
			TStart: 0, TEnd: 1,
		})
	}
	profile := momentinput.Profile{
		Outer: sectionrecord.LoopRecord{Segments: outer},
		Holes: pl.Profile0.Holes,
	}
	pp := prismrecord.Payload{Profile: profile, Frame: pl.Frame0, Z0: 0, Z1: end.Z, Xform: r3.Identity()}
	setback := capblend.Setback{Dc: d, DcDelta: dDelta, Ds: d, DsDelta: dDelta}
	source := &capblend.LoftSource{Profile: pl.Profile0, Proof: pl.Proof}
	result, err := capblend.BuildChecked(ctx, env, pp, setback, setback,
		map[int]bool{0: true}, map[int]bool{0: true}, source)
	if err != nil {
		return nil, true, fmt.Errorf("loft cap chamfer: %w", err)
	}
	return result, true, nil
}

func capRef(b *topo.Body, role string) provenance.Ref {
	return provenance.Ref{Producer: b.Origin.Producer, Role: role}
}

// HeldLoftCapChamfer is the complete outer-cap loop route of a hole-free
// solid Loft whose sections differ, such as a gear outline lofted to the
// same outline turned about its axis (agent-docs/loft-design.md §19). It
// reads the Loft's verified held mesh as a faceted shell and hands it to
// loftcylinder.ChamferLoftCaps. It reports false, with no error, for any
// other loft or selection.
func HeldLoftCapChamfer(ctx context.Context, env topo.Env, mesher verifiedmesh.Mesher, b *topo.Body,
	pl loftrecord.Payload, edges []*topo.Edge, d, dDelta float64, asym *Asymmetric) (*topo.Body, bool, error) {
	if mesher == nil || pl.SurfaceResult || !b.Solid || pl.Xform != r3.Identity() ||
		len(pl.Profile0.Holes) != 0 || len(pl.Profile1.Holes) != 0 || !pl.MirrorAxialSource() {
		return nil, false, nil
	}
	for _, offset := range pl.Alignment {
		if offset != 0 {
			return nil, false, nil
		}
	}
	n := len(pl.Verts) / 2
	if n < 3 || len(pl.Verts) != 2*n || pl.Walls != 2*n {
		return nil, false, nil
	}
	start := capRef(b, provenance.RoleCapStart)
	end := capRef(b, provenance.RoleCapEnd)
	outer := [2][]int{make([]int, n), make([]int, n)}
	for i := range n {
		outer[0][i], outer[1][i] = i, n+i
	}
	if err := loftcylinder.ChamferSelectsOuterLoops(b, start, end, edges, pl.Verts, outer); err != nil {
		// Any other selection is not this route's class; the receiver's own
		// refusal names it.
		return nil, false, nil //nolint:nilerr // a selection outside the class declines the route.
	}
	if asym != nil {
		return nil, true, fmt.Errorf("%w: asymmetric chamfer is not certified for a loft cap", decaderr.ErrUnsupported)
	}
	z0, z1 := pl.Frame0.Origin().Z, pl.Frame1.Origin().Z
	if err := loftcylinder.BindLoftCapSetback(z0, z1, d, dDelta); err != nil {
		return nil, true, err
	}
	mesh, err := mesher.VerifyAll(ctx, b, 0.1)
	if err != nil {
		return nil, true, err
	}
	if !slices.Equal(mesh.Vertices, pl.Verts) || !slices.Equal(mesh.Triangles, pl.Tris) ||
		len(mesh.Source) != len(pl.Tris) || !mesh.BoundaryOK || !mesh.SymDiffOK {
		return nil, true, fmt.Errorf("%w: the loft has no matching proven held mesh", decaderr.ErrUnsupported)
	}
	_, diameter, err := booloperand.PairChord(ctx, b, b)
	if err != nil {
		return nil, true, err
	}
	fp := facetedrecord.Payload{Verts: pl.Verts, Tris: pl.Tris, Src: make([]int, len(pl.Tris)),
		VertexBound: make([]float64, len(pl.Verts)), MeshBound: mesh.Bound, VolSymDiff: mesh.VolSymDiff,
		AreaSlack: mesh.AreaSlack, DPair: diameter, Xform: r3.Identity()}
	for i := range fp.VertexBound {
		fp.VertexBound[i] = mesh.Bound
	}
	capGroups := [2]int{-1, -1}
	groupOf := make(map[*topo.Face]int)
	for i, face := range mesh.Source {
		group, ok := groupOf[face]
		if !ok {
			group = len(fp.Groups)
			groupOf[face] = group
			g := facetedrecord.Group{Origins: append([]provenance.Ref(nil), face.Origins...)}
			for _, origin := range face.Origins {
				capIndex := -1
				switch origin {
				case start:
					capIndex = 0
				case end:
					capIndex = 1
				}
				if capIndex < 0 {
					continue
				}
				if capGroups[capIndex] >= 0 {
					return nil, true, fmt.Errorf("%w: the loft cap is split", decaderr.ErrUnsupported)
				}
				capGroups[capIndex] = group
				g.Surface, g.Planar = face.Surface, true
				g.NormalBound, g.Reversed = face.NormalBound, face.Reversed
			}
			fp.Groups = append(fp.Groups, g)
		}
		fp.Src[i] = group
	}
	if capGroups[0] < 0 || capGroups[1] < 0 {
		return nil, true, fmt.Errorf("%w: the loft has no cap faces", decaderr.ErrUnsupported)
	}
	side := loftcylinder.ChamferSideSource{Outer: outer, CapGroups: capGroups, BoreGroup: -1, Z0: z0, Z1: z1}
	body, err := loftcylinder.ChamferLoftCaps(ctx, env, fp, side, d)
	if err != nil {
		return nil, true, fmt.Errorf("loft cap chamfer: %w", err)
	}
	return body, true, nil
}
