package export

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/closedstep"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/step/ap214"
)

func admitClosedSTEP(ctx context.Context, body *decad.Body) (closedstep.Coverage, bool, error) {
	c, ok, err := closedstep.Read(ctx, body, body.Faces()[0])
	if !ok || err != nil {
		return c, ok, err
	}
	if c.Ring {
		return admitClosedRing(ctx, body, c)
	}
	pole := c.Center.Add(r3.NewVec(0, 0, c.Radius))
	if math.IsInf(pole.Z, 0) || math.IsNaN(pole.Z) {
		return closedstep.Coverage{}, false, nil
	}
	exact := proof.DyAdd(proof.MustDyOf(c.Center.Z), proof.MustDyOf(c.Radius))
	if proof.DyCmp(exact, proof.MustDyOf(pole.Z)) != 0 {
		return closedstep.Coverage{}, false, nil
	}
	normal, err := body.Faces()[0].NormalAt(pole)
	if err != nil || normal.Bound.Base() != 0 || normal.Value != r3.NewVec(0, 0, 1) {
		return closedstep.Coverage{}, false, ctx.Err()
	}
	return c, true, ctx.Err()
}
func (b *fileBuilder) addClosedSphere(c closedstep.Coverage) []step.Reference {
	a := &analyticSTEPBuilder{fileBuilder: b}
	point := a.addPoint(c.Center.Add(r3.NewVec(0, 0, c.Radius)))
	vertex := b.add(ap214.VertexPoint(0, "", point))
	loop := b.add(step.Entity{Name: "VERTEX_LOOP", Parameters: []step.Value{step.String(""), vertex}})
	bound := b.add(ap214.FaceOuterBound(0, "", loop, true))
	placement := a.addPlacement(c.Center, r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0))
	surface := b.add(ap214.SphericalSurface(0, "", placement, step.Real(c.Radius)))
	return []step.Reference{b.add(ap214.AdvancedFace(0, "", surface, true, bound))}
}

func admitClosedRing(ctx context.Context, body *decad.Body, c closedstep.Coverage) (closedstep.Coverage, bool, error) {
	sum := c.Major + c.Minor
	if math.IsInf(sum, 0) || proof.DyCmp(proof.DyAdd(proof.MustDyOf(c.Major), proof.MustDyOf(c.Minor)), proof.MustDyOf(sum)) != 0 {
		return closedstep.Coverage{}, false, nil
	}
	for _, distance := range []float64{c.Major, sum} {
		p := c.Center.Add(c.Reference.Scale(distance))
		for i, v := range []float64{p.X, p.Y, p.Z} {
			if math.IsInf(v, 0) || math.IsNaN(v) {
				return closedstep.Coverage{}, false, nil
			}
			center := []float64{c.Center.X, c.Center.Y, c.Center.Z}[i]
			ref := []float64{c.Reference.X, c.Reference.Y, c.Reference.Z}[i]
			expected := proof.DyAdd(proof.MustDyOf(center), proof.DyMul(proof.MustDyOf(ref), proof.MustDyOf(distance)))
			if proof.DyCmp(expected, proof.MustDyOf(v)) != 0 {
				return closedstep.Coverage{}, false, nil
			}
		}
	}
	p := c.Center.Add(c.Reference.Scale(sum))
	n, err := body.Faces()[0].NormalAt(p)
	if err != nil || n.Bound.Base() != 0 || n.Value != c.Reference {
		return closedstep.Coverage{}, false, ctx.Err()
	}
	return c, true, ctx.Err()
}
func (b *fileBuilder) addClosedRing(c closedstep.Coverage) []step.Reference {
	a := &analyticSTEPBuilder{fileBuilder: b}
	p := c.Center.Add(c.Reference.Scale(c.Major + c.Minor))
	tube := c.Center.Add(c.Reference.Scale(c.Major))
	point := a.addPoint(p)
	vertex := b.add(ap214.VertexPoint(0, "", point))
	surface := b.add(ap214.ToroidalSurface(0, "", a.addPlacement(c.Center, c.Axis, c.Reference), step.Real(c.Major), step.Real(c.Minor)))
	parallel := b.add(ap214.Circle(0, "", a.addPlacement(c.Center, c.Axis, c.Reference), step.Real(c.Major+c.Minor)))
	meridian := b.add(ap214.Circle(0, "", a.addPlacement(tube, c.Reference.Cross(c.Axis), c.Reference), step.Real(c.Minor)))
	pe := b.add(ap214.EdgeCurve(0, "", vertex, vertex, parallel, true))
	me := b.add(ap214.EdgeCurve(0, "", vertex, vertex, meridian, true))
	uses := []step.Reference{b.add(ap214.OrientedEdge(0, "", pe, true)), b.add(ap214.OrientedEdge(0, "", me, true)), b.add(ap214.OrientedEdge(0, "", pe, false)), b.add(ap214.OrientedEdge(0, "", me, false))}
	loop := b.add(ap214.EdgeLoop(0, "", uses...))
	bound := b.add(ap214.FaceOuterBound(0, "", loop, true))
	return []step.Reference{b.add(ap214.AdvancedFace(0, "", surface, true, bound))}
}
