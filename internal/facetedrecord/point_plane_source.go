package facetedrecord

import (
	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// PointPlaneSheet recognizes the actual step-E sheet class: tool is one
// exact planar chain sheet, and apex the point-loft apex it splits away from. A rectangle
// drawn as Patch does not pass this gate because it has a different source.
func PointPlaneSheet(tool *topo.Body, apex r3.Vec) (*PointPlaneLimit, bool) {
	cp, ok := tool.Payload.(chainrecord.Payload)
	if !ok || tool.Kind != topo.BodySheet || cp.Xform != r3.Identity() ||
		cp.Frame.Origin() != (r3.Vec{}) || cp.Frame.U() != (r3.Vec{X: 1}) ||
		cp.Frame.V() != (r3.Vec{Y: 1}) || cp.SectionDelta != 0 ||
		cp.Z0Delta != 0 || cp.Z1Delta != 0 || len(cp.Chains) != 1 ||
		len(cp.Chains[0].Segments) != 1 || len(topobuild.Faces(tool)) != 1 {
		return nil, false
	}
	line, ok := cp.Chains[0].Segments[0].(sectionrecord.LineSeg)
	if !ok || !((line.TStart == 0 && line.TEnd == 1) ||
		(line.TStart == 1 && line.TEnd == 0)) {
		return nil, false
	}
	for _, vertex := range topobuild.Vertices(tool) {
		if vertex.Bound.Mag() != 0 {
			return nil, false
		}
	}
	face := topobuild.Faces(tool)[0]
	surface, ok := face.Surface.(surfacegeom.Plane)
	if !ok || !proofbound.FiniteVec(apex) || !proofbound.FiniteVec(r3.Vec{
		X: line.Start.U, Y: line.Start.V, Z: line.End.U}) ||
		!proofbound.FiniteVec(r3.Vec{X: line.End.V, Y: cp.Z0, Z: cp.Z1}) {
		return nil, false
	}
	ax, ay := ratOfPointPlane(line.Start.U), ratOfPointPlane(line.Start.V)
	bx, by := ratOfPointPlane(line.End.U), ratOfPointPlane(line.End.V)
	nx := pointPlaneSub(ay, by)
	ny := pointPlaneSub(bx, ax)
	if nx.Sign() == 0 && ny.Sign() == 0 {
		return nil, false
	}
	d := pointPlaneAdd(
		pointPlaneMul(nx, pointPlaneSub(ax, ratOfPointPlane(apex.X))),
		pointPlaneMul(ny, pointPlaneSub(ay, ratOfPointPlane(apex.Y))),
	)
	if d.Sign() == 0 {
		return nil, false
	}
	if d.Sign() < 0 {
		nx.Neg(nx)
		ny.Neg(ny)
		d.Neg(d)
	}
	// Sketch may canonicalize a one-line chain independently of CreateLine's
	// endpoint order. Compare the actual face frame to the geometric plane
	// after orienting that plane away from the apex.
	frameNormal := surface.Frame.N()
	if !proofbound.FiniteVec(frameNormal) || proofbound.IsNonFinite(face.NormalBound) ||
		face.NormalBound < 0 {
		return nil, false
	}
	dot := pointPlaneAdd(pointPlaneMul(nx, ratOfPointPlane(frameNormal.X)),
		pointPlaneMul(ny, ratOfPointPlane(frameNormal.Y)))
	allow := pointPlaneMul(pointPlaneAdd(pointPlaneAbs(nx), pointPlaneAbs(ny)),
		ratOfPointPlane(face.NormalBound))
	if pointPlaneAbs(dot).Cmp(allow) <= 0 {
		return nil, false
	}
	return &PointPlaneLimit{Nx: nx, Ny: ny, D: d,
		A: line.Start, B: line.End, Z0: cp.Z0, Z1: cp.Z1,
		Surface: surface, Origins: append([]provenance.Ref(nil), face.Origins...), NormalBound: face.NormalBound,
		FrameTowardFar: dot.Sign() > 0}, true
}

// PointPlaneSource is the slab record a point-plane Split cuts: target's
// own record when it is already a slab, else its whole authenticated
// point-loft fan.
func PointPlaneSource(target *topo.Body) (PointPlaneSliceRecord, bool) {
	fp, ok := target.Payload.(Payload)
	if !ok || target.Kind != topo.BodySolid || !target.Solid ||
		!fp.PointPlaneSourceEligible || fp.Xform != r3.Identity() {
		return PointPlaneSliceRecord{}, false
	}
	if fp.PointPlane != nil {
		return *fp.PointPlane, true
	}
	if fp.PointSection == nil {
		return PointPlaneSliceRecord{}, false
	}
	return PointPlaneSliceRecord{Base: fp}, true
}
