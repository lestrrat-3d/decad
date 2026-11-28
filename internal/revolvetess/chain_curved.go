package revolvetess

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/revolvesampling"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// tessellateChainCircular uses certified circular meridian and angular stations.
func tessellateChainCircular(ctx context.Context, body *topo.Body, chain revolveplan.ChainPayload, chord float64, verify Level, sweepRoles bool) (*topo.Mesh, error) {
	bad := func() (*topo.Mesh, error) {
		return nil, fmt.Errorf("%w: circular chain revolve has no certified mesh at this tolerance", decaderr.ErrUnsupported)
	}
	if body.Kind != topo.BodySheet || len(chain.Chains) != 1 || chain.SectionDelta != 0 ||
		(chain.Ax.RadialLower <= 0 && (!chain.Ax.RadialProof || len(chain.Chains[0].Segments) != 1)) {
		return bad()
	}
	for _, seg := range chain.Chains[0].Segments {
		switch seg.(type) {
		case sectionrecord.ArcSeg, lineSeg:
		default:
			return bad()
		}
	}
	rp := chain.Revolve()
	resolved, err := revolveaxis.ResolveChain(ctx, chain.Chains[0], freeform.NewFreeformWork(), "circular chain revolve", rp.ChargedWalk, rp.Ax.SnapTol)
	if err != nil {
		return nil, err
	}
	if len(resolved.Walks) == 0 {
		return bad()
	}
	budget := proofbound.NewWorkBudget(ctx)
	lift := rp.Lift()
	facesByRole := map[string]*topo.Face{}
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			if origin.Producer == body.Origin.Producer {
				facesByRole[origin.Role] = face
			}
		}
	}
	type planeStation struct {
		u, v  float64
		bound proofbound.WalkEndBound
	}
	var meridians []revolvemesh.RevMeridian
	var source []planeStation
	var cellFaces []*topo.Face
	var faces []*topo.Face
	meridianSag, radius := 0.0, 0.0
	for wi, walk := range resolved.Walks {
		if walk.Closed || resolved.Kinds[wi] == revolveaxis.WallAxis {
			return bad()
		}
		role := fmt.Sprintf("side(0,%d)", walk.Segs[0])
		if sweepRoles {
			role = fmt.Sprintf("side(0,0,%d)", walk.Segs[0])
		}
		face := facesByRole[role]
		if face == nil {
			return bad()
		}
		faces = append(faces, face)
		for _, segIndex := range walk.Segs {
			plane := resolved.Plane[segIndex]
			axisWalk, err := rp.ChargedWalk(resolved.Segs[segIndex], plane)
			if err != nil {
				return nil, err
			}
			bound := boundarywalk.DenotedStartBound(resolved.Segs[segIndex], plane)
			if segIndex > 0 {
				bound = boundarywalk.JunctionStartBound(resolved.Segs[segIndex-1], resolved.Plane[segIndex-1], resolved.Segs[segIndex], plane)
			}
			zi, ri, ok := revolvemesh.RevolveMeridianEnclosure(lift.AU, lift.AV, lift.DU, lift.DV, plane.StartU, plane.StartV, bound)
			onAxis := axisWalk.StartV == 0
			if !ok || axisWalk.StartV < 0 || (!onAxis && ri.Lo.Sign() <= 0) ||
				(onAxis && (segIndex != 0 || ri.Lo.Sign() != 0 || ri.Hi.Sign() != 0)) {
				return bad()
			}
			if onAxis {
				_, trueRho, ok := revolveproof.ChainArcSourcePoint(rp.Ax, plane.StartU, plane.StartV, bound)
				if !ok || trueRho.Lo.Sign() != 0 || trueRho.Hi.Sign() != 0 {
					return bad()
				}
			}
			station := revolvemesh.RevMeridian{Z: axisWalk.StartU, Rho: axisWalk.StartV, ZIv: zi, RhoIv: ri, OnAxis: onAxis}
			point := planeStation{u: plane.StartU, v: plane.StartV, bound: bound}
			count, sag := 1, 0.0
			if plane.IsCircular() {
				count, sag, err = tessellation.ChordCountWithBudget(plane, chord/8, revolvesampling.MeridianMin(plane), budget)
				if err != nil {
					return nil, err
				}
			}
			meridianSag = math.Max(meridianSag, sag)
			radius = math.Max(radius, rp.Ax.RadialUpper(plane.CoordUpper))
			for i := range count {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				if i > 0 {
					station, _, err = revolvesampling.ArcStation(lift, resolved.Segs[segIndex], i, count)
					if err != nil {
						return nil, err
					}
					if station.RhoIv.Lo.Sign() <= 0 {
						return bad()
					}
					point.u, point.v, point.bound, err = revolvesampling.ArcPlaneStation(resolved.Segs[segIndex], i, count)
					if err != nil {
						return nil, err
					}
				}
				station.Sag = sag
				meridians = append(meridians, station)
				source = append(source, point)
				cellFaces = append(cellFaces, face)
			}
		}
	}
	last := len(resolved.Segs) - 1
	plane := resolved.Plane[last]
	axisWalk, err := rp.ChargedWalk(resolved.Segs[last], plane)
	if err != nil {
		return nil, err
	}
	zi, ri, ok := revolvemesh.RevolveMeridianEnclosure(lift.AU, lift.AV, lift.DU, lift.DV, plane.EndU, plane.EndV,
		boundarywalk.DenotedEndBound(resolved.Segs[last], plane))
	onAxis := axisWalk.EndV == 0
	if !ok || axisWalk.EndV < 0 || (!onAxis && ri.Lo.Sign() <= 0) ||
		(onAxis && (ri.Lo.Sign() != 0 || ri.Hi.Sign() != 0 || meridians[0].OnAxis)) {
		return bad()
	}
	// With no positive clearance, a single circular arc must have an exact
	// free pole and a proven non-negative radius over the whole arc. A circle
	// cannot touch the axis again in its interior without crossing it, except
	// at a tangent point, which is unique on this open circular walk.
	if chain.Ax.RadialLower <= 0 && !onAxis && !meridians[0].OnAxis {
		return bad()
	}
	if onAxis {
		_, trueRho, ok := revolveproof.ChainArcSourcePoint(rp.Ax, plane.EndU, plane.EndV, boundarywalk.DenotedEndBound(resolved.Segs[last], plane))
		if !ok || trueRho.Lo.Sign() != 0 || trueRho.Hi.Sign() != 0 {
			return bad()
		}
	}
	meridians = append(meridians, revolvemesh.RevMeridian{Z: axisWalk.EndU, Rho: axisWalk.EndV, ZIv: zi, RhoIv: ri, OnAxis: onAxis})
	source = append(source, planeStation{u: plane.EndU, v: plane.EndV, bound: boundarywalk.DenotedEndBound(resolved.Segs[last], plane)})
	angle0, ok0 := rp.Den.Phi0.Enclosure()
	angle1, ok1 := rp.Den.Phi1.Enclosure()
	if !ok0 || !ok1 {
		return bad()
	}
	span := proofbound.IntervalSub(angle1, angle0)
	if span.Lo.Sign() <= 0 {
		return bad()
	}
	sweep := proofbound.RatFloatUp(span.Hi)
	ideal, ok := revolvemesh.IdealBasis(lift.Frame, lift.AU, lift.AV, lift.DU, lift.DV)
	if !ok || proofbound.IsNonFinite(radius) || proofbound.IsNonFinite(sweep) {
		return bad()
	}
	// A true unit axis only rotates the source plane coordinates. PlaneMap
	// bounds the frame and placement stretch for every such rotation.
	rotation, err := massmoment.PlaneMap(lift.Frame, rp.Xform)
	if err != nil {
		return nil, err
	}
	charge, err := massmoment.MapChargeOf(rotation)
	if err != nil {
		return nil, err
	}
	stretch := proofbound.AbsSumUpper(1, charge.Stretch)
	var work revolveWork
	for n := 8; n <= maxChainSweepArcCells; n *= 2 {
		angularSag := tessellation.ChordSagitta(radius, sweep, n)
		if proofbound.IsNonFinite(angularSag) || angularSag > chord/4 {
			continue
		}
		if err := revolvePreflightFacets([]revLoopMesh{{resolved: resolved, samples: meridians}}, n, rp.Full, true, &work); err != nil {
			return nil, err
		}
		angular, err := revolvemesh.AngularSequence(revolvemesh.AngularInput{Phi0: rp.Phi0, Phi1: rp.Phi1, Full: rp.Full, Den: rp.Den}, n)
		if err != nil {
			return nil, err
		}
		verts, dc, dr, err := revolvemesh.EmitRings([][]revolvemesh.RevMeridian{meridians}, angular, lift.Basis(), ideal, rp.Xform, rp.Full, budget)
		if err != nil {
			return nil, err
		}
		coord := proofbound.AbsSumUpper(dc, dr)
		// The ring builder bounds the held axis. Compare every station with
		// the recorded axis as well, including its direction uncertainty.
		for j, point := range source {
			for l := range angular.Samples {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				gap := lift.SweptPointGap(rp.AxisBound(), rp.Xform, point.u, point.v, point.bound,
					angular.SinIv[l], angular.CosIv[l], verts[meridians[j].At(l)])
				coord = math.Max(coord, gap)
			}
		}
		bound := proofbound.AbsSumUpper(proofbound.ProductUpper(meridianSag, stretch),
			proofbound.ProductUpper(angularSag, stretch), coord)
		if proofbound.IsNonFinite(bound) || bound > chord {
			continue
		}
		mesh := &topo.Mesh{Vertices: verts, CoordBound: coord}
		for i, face := range cellFaces {
			revolvemesh.EmitCellTriangles(meridians[i], meridians[i+1], n, func(tri [3]int) { topobuild.AddTriangle(mesh, tri, face) })
		}
		if rp.Reflected() {
			for i := range mesh.Triangles {
				mesh.Triangles[i][1], mesh.Triangles[i][2] = mesh.Triangles[i][2], mesh.Triangles[i][1]
			}
		}
		for _, face := range faces {
			topobuild.SetFaceBound(mesh, face, bound)
		}
		if err := topobuild.RequireSheetMesh(ctx, body, mesh); err != nil {
			return nil, err
		}
		audit, err := revolvemesh.RequireRevolveFacetAreas(budget, mesh.Vertices, mesh.Triangles, coord)
		if err != nil {
			return nil, topobuild.LiftAuditError(err)
		}
		if verify >= VerifyBoundary {
			if err := revolvemesh.RevolveContactAuditBounded(budget, audit, mesh.Triangles, coord); err != nil {
				return nil, topobuild.LiftAuditError(err)
			}
		}
		if verify >= VerifyAll {
			areas := make(map[*topo.Face]proofbound.RatInterval, len(faces))
			for _, face := range faces {
				areas[face] = proofbound.RatInterval{Lo: new(big.Rat), Hi: new(big.Rat)}
			}
			for i, tri := range mesh.Triangles {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				lo, hi, ok := tessellation.ChainSweepTriangleArea(verts, tri)
				if !ok {
					return bad()
				}
				face := mesh.Source[i]
				area := areas[face]
				area.Lo.Add(area.Lo, proofarith.FloatRat(lo))
				area.Hi.Add(area.Hi, proofarith.FloatRat(hi))
			}
			for _, face := range faces {
				slack, err := tessellation.ChainSweepFaceAreaSlack(face.Area, face.AreaBound, areas[face])
				if err != nil {
					return nil, err
				}
				mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, slack)
			}
		}

		return mesh, nil
	}
	return bad()
}
