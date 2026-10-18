package clearance

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// BuildStitchBodyCarriers reads the live stitched faces in their body order.
// The caller has already checked the triangle-set and vertex-bound gates.
func BuildStitchBodyCarriers(budget *proofbound.WorkBudget, faces []*topo.Face,
	sp stitchrecord.Payload) ([]*CFace, bool, error) {
	var carriers []*CFace
	for _, f := range faces {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		pl, ok := f.Surface.(surfacegeom.Plane)
		if !ok {
			return nil, false, nil
		}
		normal := pl.Frame.N()
		if f.Reversed {
			normal = normal.Scale(-1)
		}
		cf := &CFace{Kind: CkPlane, O: pl.Frame.Origin(), U: pl.Frame.U(), V: pl.Frame.V(), N: normal}

		var elems []survey2d.SurveyElem
		var pts []r3.Vec
		for _, l := range f.Loops {
			cnt := len(l.CoEdges)
			if cnt == 0 {
				return nil, false, nil
			}
			local := make([]r3.Vec, cnt)
			for i, ce := range l.CoEdges {
				if err := budget.Step(); err != nil {
					return nil, false, err
				}
				v := topobuild.CoEdgeStart(ce).Position
				pts = append(pts, v)
				local[i] = pl.Frame.ToLocal(v)
			}
			for i := range local {
				a, next := local[i], local[(i+1)%cnt]
				if el, ok := survey2d.LineElem(a.X, a.Y, next.X, next.Y); ok {
					elems = append(elems, el)
				}
			}
		}
		region, err := NewRegion2Budget(budget, elems)
		if err != nil {
			return nil, false, err
		}
		cf.Region = region
		cf.Box = BoxUnion(BoxOf(pts...), CapBox(cf))

		for i, tri := range sp.Tris {
			if sp.TriFaces[i] != f {
				continue
			}
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			a, b, c := sp.Verts[tri[0]], sp.Verts[tri[1]], sp.Verts[tri[2]]
			cen := a.Add(b).Add(c).Scale(1.0 / 3.0)
			lx, ly := cf.PlaneCoords(cen)
			if cf.Region.Classify(lx, ly, cf.Region.Tol()) > 0 {
				cf.Wit = append(cf.Wit, cen)
			}
		}
		carriers = append(carriers, cf)
	}
	return carriers, true, nil
}
