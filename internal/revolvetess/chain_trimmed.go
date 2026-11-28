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

type trimmedChain struct {
	resolved revolveaxis.ResolvedWalks
	stations []chainStation
	samples  []revolvemesh.RevMeridian
	faces    []*topo.Face
}

// tessellateTrimmedChains shares one angular sequence across open chains.
func tessellateTrimmedChains(ctx context.Context, body *topo.Body, chain revolveplan.ChainPayload, chord float64, verify Level) (*topo.Mesh, error) {
	bad := func() (*topo.Mesh, error) {
		return nil, fmt.Errorf("%w: trimmed revolve chains need off-axis line or circular walls with bounded cuts", decaderr.ErrUnsupported)
	}
	if body.Kind != topo.BodySheet || len(chain.Chains) == 0 || chain.SectionDelta < 0 || proofbound.IsNonFinite(chain.SectionDelta) {
		return bad()
	}
	rp := chain.Revolve()
	lift := rp.Lift()
	facesByRole := map[string]*topo.Face{}
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			if origin.Producer == body.Origin.Producer {
				facesByRole[origin.Role] = face
			}
		}
	}
	chains := make([]trimmedChain, len(chain.Chains))
	maxRadius, maxSag := 0.0, 0.0
	budget := proofbound.NewWorkBudget(ctx)
	for ci, record := range chain.Chains {
		for _, seg := range record.Segments {
			switch seg.(type) {
			case lineSeg:
			case sectionrecord.ArcSeg, sectionrecord.CircleSeg:
				// The original axis clearance covers the whole source curve.
				// The cut displacement may widen either plane coordinate.
				if chain.Ax.RadialLower <= proofbound.ProductUpper(2, chain.SectionDelta) {
					return bad()
				}
			default:
				return bad()
			}
		}
		resolved, err := revolveaxis.ResolveChain(ctx, record, freeform.NewFreeformWork(), "trimmed chain revolve", rp.ChargedWalk, rp.Ax.SnapTol)
		if err != nil {
			return nil, err
		}
		walks := resolved.Walks
		if len(walks) == 0 {
			return bad()
		}
		out := &chains[ci]
		out.resolved = resolved
		for wi, walk := range walks {
			if walk.Closed || resolved.Kinds[wi] == revolveaxis.WallAxis {
				return bad()
			}
			face := facesByRole[fmt.Sprintf("side(%d,%d)", ci, walk.Segs[0])]
			if face == nil {
				return bad()
			}
			for _, index := range walk.Segs {
				plane := resolved.Plane[index]
				axisWalk, err := rp.ChargedWalk(resolved.Segs[index], plane)
				if err != nil {
					return nil, err
				}
				bound := boundarywalk.DenotedStartBound(resolved.Segs[index], plane)
				if index > 0 {
					bound = boundarywalk.JunctionStartBound(resolved.Segs[index-1], resolved.Plane[index-1], resolved.Segs[index], plane)
				}
				bound.U = proofbound.AbsSumUpper(bound.U, chain.SectionDelta)
				bound.V = proofbound.AbsSumUpper(bound.V, chain.SectionDelta)
				station := chainStation{u: plane.StartU, v: plane.StartV, bound: bound}
				zi, ri, ok := revolvemesh.RevolveMeridianEnclosure(lift.AU, lift.AV, lift.DU, lift.DV, station.u, station.v, station.bound)
				if !ok || axisWalk.StartV <= 0 || ri.Lo.Sign() <= 0 {
					return bad()
				}
				sample := revolvemesh.RevMeridian{Z: axisWalk.StartU, Rho: axisWalk.StartV, ZIv: zi, RhoIv: ri}
				count, sag := 1, 0.0
				if plane.IsCircular() {
					count, sag, err = tessellation.ChordCountWithBudget(plane, chord/8, revolvesampling.MeridianMin(plane), budget)
					if err != nil {
						return nil, err
					}
				}
				maxSag = math.Max(maxSag, sag)
				maxRadius = math.Max(maxRadius, rp.Ax.RadialUpper(revolveaxis.SectionCoordUpper(plane.CoordUpper, chain.SectionDelta)))
				for j := range count {
					if err := budget.Step(); err != nil {
						return nil, err
					}
					if j > 0 {
						sample, _, err = revolvesampling.ArcStation(lift, resolved.Segs[index], j, count)
						if err != nil {
							return nil, err
						}
						station.u, station.v, station.bound, err = revolvesampling.ArcPlaneStation(resolved.Segs[index], j, count)
						if err != nil {
							return nil, err
						}
						station.bound.U = proofbound.AbsSumUpper(station.bound.U, chain.SectionDelta)
						station.bound.V = proofbound.AbsSumUpper(station.bound.V, chain.SectionDelta)
					}
					_, rho, ok := revolveproof.ChainArcSourcePoint(rp.Ax, station.u, station.v, station.bound)
					if !ok || rho.Lo.Sign() <= 0 || sample.RhoIv.Lo.Sign() <= 0 {
						return bad()
					}
					out.stations = append(out.stations, station)
					out.samples = append(out.samples, sample)
					out.faces = append(out.faces, face)
				}
			}
		}
		last := len(resolved.Segs) - 1
		plane := resolved.Plane[last]
		axisWalk, err := rp.ChargedWalk(resolved.Segs[last], plane)
		if err != nil {
			return nil, err
		}
		bound := boundarywalk.DenotedEndBound(resolved.Segs[last], plane)
		bound.U = proofbound.AbsSumUpper(bound.U, chain.SectionDelta)
		bound.V = proofbound.AbsSumUpper(bound.V, chain.SectionDelta)
		station := chainStation{u: plane.EndU, v: plane.EndV, bound: bound}
		zi, ri, ok := revolvemesh.RevolveMeridianEnclosure(lift.AU, lift.AV, lift.DU, lift.DV, station.u, station.v, bound)
		_, trueR, sourceOK := revolveproof.ChainArcSourcePoint(rp.Ax, station.u, station.v, bound)
		if !ok || !sourceOK || axisWalk.EndV <= 0 || ri.Lo.Sign() <= 0 || trueR.Lo.Sign() <= 0 {
			return bad()
		}
		out.stations = append(out.stations, station)
		out.samples = append(out.samples, revolvemesh.RevMeridian{Z: axisWalk.EndU, Rho: axisWalk.EndV, ZIv: zi, RhoIv: ri})
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
	var work revolveWork
	for n := 8; n <= maxChainSweepArcCells; n *= 2 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sag := tessellation.ChordSagitta(maxRadius, sweep, n)
		if proofbound.IsNonFinite(sag) || sag >= chord {
			continue
		}
		inputs := make([][]revolvemesh.RevMeridian, len(chains))
		preflight := make([]revLoopMesh, len(chains))
		for i := range chains {
			inputs[i] = chains[i].samples
			preflight[i] = revLoopMesh{resolved: chains[i].resolved, samples: inputs[i]}
		}
		if err := revolvePreflightFacets(preflight, n, rp.Full, true, &work); err != nil {
			return nil, err
		}
		angular, err := revolvemesh.AngularSequence(revolvemesh.AngularInput{Phi0: rp.Phi0, Phi1: rp.Phi1, Full: rp.Full, Den: rp.Den}, n)
		if err != nil {
			return nil, err
		}
		budget := proofbound.NewWorkBudget(ctx)
		verts, dc, dr, err := revolvemesh.EmitRings(inputs, angular, lift.Basis(), ideal, rp.Xform, rp.Full, budget)
		if err != nil {
			return nil, err
		}
		coord := proofbound.AbsSumUpper(dc, dr)
		for i, c := range chains {
			for j, station := range c.stations {
				for l := range angular.Samples {
					if err := budget.Step(); err != nil {
						return nil, err
					}
					gap := lift.SweptPointGap(rp.AxisBound(), rp.Xform, station.u, station.v, station.bound,
						angular.SinIv[l], angular.CosIv[l], verts[inputs[i][j].At(l)])
					coord = math.Max(coord, gap)
				}
			}
		}
		bound := proofbound.AbsSumUpper(proofbound.ProductUpper(proofbound.AbsSumUpper(sag, maxSag), stretch), coord)
		if proofbound.IsNonFinite(bound) || bound > chord {
			continue
		}
		mesh := &topo.Mesh{Vertices: verts, CoordBound: coord}
		for i, c := range chains {
			for j, face := range c.faces {
				revolvemesh.EmitCellTriangles(inputs[i][j], inputs[i][j+1], n, func(tri [3]int) { topobuild.AddTriangle(mesh, tri, face) })
				topobuild.SetFaceBound(mesh, face, bound)
			}
		}
		if rp.Reflected() {
			for i := range mesh.Triangles {
				mesh.Triangles[i][1], mesh.Triangles[i][2] = mesh.Triangles[i][2], mesh.Triangles[i][1]
			}
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
			areas := map[*topo.Face]proofbound.RatInterval{}
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
				if area.Lo == nil {
					area = proofbound.RatInterval{Lo: new(big.Rat), Hi: new(big.Rat)}
				}
				area.Lo.Add(area.Lo, proofarith.FloatRat(lo))
				area.Hi.Add(area.Hi, proofarith.FloatRat(hi))
				areas[face] = area
			}
			seen := make(map[*topo.Face]bool, len(areas))
			for _, c := range chains {
				for _, face := range c.faces {
					if seen[face] {
						continue
					}
					seen[face] = true
					slack, err := tessellation.ChainSweepFaceAreaSlack(face.Area, face.AreaBound, areas[face])
					if err != nil {
						return nil, err
					}
					mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, slack)
				}
			}
		}
		return mesh, nil
	}
	return bad()
}
