package revolvetess

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

type chainStation struct {
	u, v         float64
	bound        proofbound.WalkEndBound
	trueZ, trueR proofbound.RatInterval
}

// tessellateChainLines shares one angular grid at every junction of a straight-walk chain.
func tessellateChainLines(ctx context.Context, body *topo.Body, chain revolveplan.ChainPayload, chord float64, verify Level, sweepRoles, allowPole bool) (*topo.Mesh, error) {
	bad := func() (*topo.Mesh, error) {
		return nil, fmt.Errorf("%w: the revolve chain needs connected straight walls with proven junctions", decaderr.ErrUnsupported)
	}
	if chain.SectionDelta != 0 || len(chain.Chains) != 1 {
		return bad()
	}
	for _, segment := range chain.Chains[0].Segments {
		if _, ok := segment.(lineSeg); !ok {
			return bad()
		}
	}
	rp := chain.Revolve()
	resolved, err := revolveaxis.ResolveChain(ctx, chain.Chains[0], freeform.NewFreeformWork(), "chain Revolve tessellation", rp.ChargedWalk, rp.Ax.SnapTol)
	if err != nil {
		return nil, err
	}
	walks := resolved.Walks
	if len(walks) == 0 {
		return bad()
	}
	lift := rp.Lift()
	stations := make([]chainStation, len(walks)+1)
	meridians := make([]revolvemesh.RevMeridian, len(stations))
	maxRadius := 0.0
	for i := range stations {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		var z, rho float64
		if i == len(walks) {
			w := walks[i-1]
			last := w.Segs[len(w.Segs)-1]
			p := resolved.Plane[last]
			stations[i].u, stations[i].v = p.EndU, p.EndV
			stations[i].bound = boundarywalk.DenotedEndBound(resolved.Segs[last], p)
			z, rho = w.EndU, w.EndV
		} else {
			w := walks[i]
			first := w.Segs[0]
			p := resolved.Plane[first]
			stations[i].u, stations[i].v = p.StartU, p.StartV
			stations[i].bound = boundarywalk.DenotedStartBound(resolved.Segs[first], p)
			if i > 0 {
				prev := walks[i-1]
				last := prev.Segs[len(prev.Segs)-1]
				stations[i].bound = boundarywalk.JunctionStartBound(resolved.Segs[last], resolved.Plane[last], resolved.Segs[first], p)
			}
			z, rho = w.StartU, w.StartV
		}
		if rho < 0 || (rho == 0 && (!allowPole || (i != 0 && i != len(walks)))) {
			return bad()
		}
		station := &stations[i]
		var ok bool
		station.trueZ, station.trueR, ok = revolveproof.ChainArcSourcePoint(rp.Ax, station.u, station.v, station.bound)
		if !ok || (rho > 0 && station.trueR.Lo.Sign() <= 0) || (rho == 0 && (station.trueR.Lo.Sign() != 0 || station.trueR.Hi.Sign() != 0)) {
			return bad()
		}
		ziv, riv, ok := revolvemesh.RevolveMeridianEnclosure(lift.AU, lift.AV, lift.DU, lift.DV, station.u, station.v, station.bound)
		if !ok || (rho > 0 && riv.Lo.Sign() <= 0) || (rho == 0 && (riv.Lo.Sign() != 0 || riv.Hi.Sign() != 0)) {
			return bad()
		}
		meridians[i] = revolvemesh.RevMeridian{Z: z, Rho: rho, ZIv: ziv, RhoIv: riv, OnAxis: rho == 0}
		maxRadius = math.Max(maxRadius, proofbound.RatFloatUp(station.trueR.Hi))
	}
	facesByRole := map[string]*topo.Face{}
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			if origin.Producer == body.Origin.Producer {
				facesByRole[origin.Role] = face
			}
		}
	}
	faces := make([]*topo.Face, len(walks))
	for i, walk := range walks {
		if resolved.Kinds[i] == revolveaxis.WallAxis {
			return bad()
		}
		role := fmt.Sprintf("side(0,%d)", walk.Segs[0])
		if sweepRoles {
			role = fmt.Sprintf("side(0,0,%d)", walk.Segs[0])
		}
		faces[i] = facesByRole[role]
		if faces[i] == nil {
			return bad()
		}
	}
	angle0, ok0 := rp.Den.Phi0.Enclosure()
	angle1, ok1 := rp.Den.Phi1.Enclosure()
	if !ok0 || !ok1 {
		return bad()
	}
	span := proofbound.IntervalSub(angle1, angle0)
	if span.Lo.Sign() <= 0 || proofbound.IsNonFinite(maxRadius) {
		return bad()
	}
	sweep := proofbound.RatFloatUp(span.Hi)
	ideal, ok := revolvemesh.IdealBasis(lift.Frame, lift.AU, lift.AV, lift.DU, lift.DV)
	if !ok {
		return bad()
	}
	mapping, err := massmoment.PlaneMap(lift.Frame, rp.Xform)
	if err != nil {
		return nil, err
	}
	charge, err := massmoment.MapChargeOf(mapping)
	if err != nil {
		return nil, err
	}
	stretch := proofbound.AbsSumUpper(1, charge.Stretch)
	for n := 8; n <= maxChainSweepArcCells; n *= 2 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sag := tessellation.ChordSagitta(maxRadius, sweep, n)
		if proofbound.IsNonFinite(sag) || sag >= chord {
			continue
		}
		angular, err := revolvemesh.AngularSequence(revolvemesh.AngularInput{Phi0: rp.Phi0, Phi1: rp.Phi1, Full: rp.Full, Den: rp.Den}, n)
		if err != nil {
			return nil, err
		}
		budget := proofbound.NewWorkBudget(ctx)
		verts, dc, dr, err := revolvemesh.EmitRings([][]revolvemesh.RevMeridian{meridians}, angular, lift.Basis(), ideal, rp.Xform, rp.Full, budget)
		if err != nil {
			return nil, err
		}
		coord := proofbound.AbsSumUpper(dc, dr)
		vertexGap := 0.0
		for j, station := range stations {
			for l := range angular.Samples {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				gap := lift.SweptPointGap(rp.AxisBound(), rp.Xform, station.u, station.v, station.bound,
					angular.SinIv[l], angular.CosIv[l], verts[meridians[j].At(l)])
				vertexGap = math.Max(vertexGap, gap)
			}
		}
		bound := proofbound.AbsSumUpper(proofbound.ProductUpper(sag, stretch), vertexGap)
		if proofbound.IsNonFinite(bound) || bound > chord {
			continue
		}
		mesh := &topo.Mesh{Vertices: verts, CoordBound: math.Max(coord, vertexGap)}
		for i, face := range faces {
			revolvemesh.EmitCellTriangles(meridians[i], meridians[i+1], n, func(tri [3]int) { topobuild.AddTriangle(mesh, tri, face) })
			topobuild.SetFaceBound(mesh, face, bound)
		}
		if rp.Reflected() {
			for i := range mesh.Triangles {
				mesh.Triangles[i][1], mesh.Triangles[i][2] = mesh.Triangles[i][2], mesh.Triangles[i][1]
			}
		}
		if err := topobuild.RequireSheetMesh(ctx, body, mesh); err != nil {
			return nil, err
		}
		audit, err := revolvemesh.RequireRevolveFacetAreas(budget, mesh.Vertices, mesh.Triangles, mesh.CoordBound)
		if err != nil {
			return nil, topobuild.LiftAuditError(err)
		}
		if verify >= VerifyBoundary {
			if err := revolvemesh.RevolveContactAudit(budget, audit, mesh.Triangles, mesh.CoordBound); err != nil {
				return nil, topobuild.LiftAuditError(err)
			}
		}
		if verify >= VerifyAll {
			for i := range walks {
				a, z := stations[i], stations[i+1]
				axis, err := revolveproof.ChainArcAreaDensityAllow(a.trueZ, a.trueR, z.trueZ, z.trueR, meridians[i].Z, meridians[i].Rho, meridians[i+1].Z, meridians[i+1].Rho, span, angular.Step, n)
				if err != nil {
					return nil, err
				}
				cell, err := revolveproof.CellSlack(ideal, angular, meridians[i], meridians[i+1], coord)
				if err != nil {
					return nil, err
				}
				mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, proofbound.ProductUpper(float64(n), proofbound.AbsSumUpper(cell, axis)))
			}
			mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, revolvemesh.CoordinateAreaAllow(mesh.Vertices, mesh.Triangles, coord))
			if proofbound.IsNonFinite(mesh.AreaSlack) {
				return bad()
			}
		}
		return mesh, nil
	}
	return bad()
}
