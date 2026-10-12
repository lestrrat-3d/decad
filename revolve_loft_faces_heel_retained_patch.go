package decad

import (
	"context"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// The two retained heel-cone patches share their contact-circle vertices
// with the source-aligned blank shell. They are open construction pieces;
// adjacent LoftFaces walls and occupied-volume proof are still required.
type revolveLoftFacesHeelRetainedPatches struct {
	shell      *revolveLoftFacesBlankSeamShell
	mesh       joinedMesh
	bounds     []float64
	rows       [2][][]int
	chord      float64
	curveBound float64
	patchBound float64
	heel       *pointConeLimit
}

func revolveLoftFacesHeelRetainedPatchesOf(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, budget *big.Rat) (
	*revolveLoftFacesHeelRetainedPatches, bool, error,
) {
	if ctx == nil || proof == nil || budget == nil || budget.Sign() <= 0 {
		return nil, false, nil
	}
	tracks, ok, err := revolveLoftFacesHeelRetainedTracksOf(ctx, proof, budget)
	if err != nil || !ok || tracks == nil {
		return nil, false, err
	}
	manifest, ok, err := revolveLoftFacesAllSideManifest(ctx, proof)
	if err != nil || !ok || manifest == nil {
		return nil, false, err
	}
	sectionIndex, cellIndex := -1, -1
	for _, side := range manifest.nonvertical {
		for index, cell := range side.cells {
			if cell.mapped.side != revolveLoftFacesJointContact ||
				cell.mapped.jointKind != revolveLoftFacesWholeArcContact {
				continue
			}
			if sectionIndex >= 0 {
				continue
			}
			sectionIndex, cellIndex = side.sectionIndex, index
		}
	}
	if sectionIndex < 0 {
		return nil, false, nil
	}
	shell, ok, err := revolveLoftFacesBlankShellWithCapRings(ctx, proof,
		sectionIndex, cellIndex)
	if err != nil || !ok || shell == nil || len(shell.capRings) != 2 {
		return nil, false, err
	}
	birth, ok, err := certifyRevolveLoftFacesHeelBirth(ctx, proof)
	if err != nil || !ok || birth == nil {
		return nil, false, err
	}
	terminal, ok, err := certifyRevolveLoftFacesHeelTerminal(ctx, proof)
	if err != nil || !ok || terminal == nil ||
		birth.capFace == nil || birth.capFace != terminal.capFace {
		return nil, false, err
	}
	var rings [2]*revolveLoftFacesShellCapRing
	for index := range shell.capRings {
		ring := &shell.capRings[index]
		switch {
		case revolveLoftFacesToolCapOccupiedArcEqual(ring.arc, birth.lower):
			if rings[0] != nil {
				return nil, false, nil
			}
			rings[0] = ring
		case revolveLoftFacesToolCapOccupiedArcEqual(ring.arc, terminal.upper):
			if rings[1] != nil {
				return nil, false, nil
			}
			rings[1] = ring
		default:
			return nil, false, nil
		}
	}
	if rings[0] == nil || rings[1] == nil {
		return nil, false, nil
	}
	out := &revolveLoftFacesHeelRetainedPatches{shell: shell,
		mesh: joinedMesh{verts: append([]r3.Vec(nil), shell.mesh.verts...),
			groups: []facetGroup{{origins: birth.capFace.Origins(),
				surface: birth.capFace.Surface(), reversed: birth.capFace.reversed}}},
		bounds: append([]float64(nil), shell.vertexBounds...), heel: proof.trim.heel}
	for side, track := range []*revolveLoftFacesHeelRetainedTrack{
		tracks.birth, tracks.terminal} {
		if track == nil || len(track.arc) < 2 || len(track.stations) < 2 {
			return nil, false, nil
		}
		arcRow := make([]int, 0, len(track.arc))
		for _, vertex := range track.arc {
			id := out.add(vertex.point, max(vertex.loftBound, vertex.blankBound))
			arcRow = append(arcRow, id)
		}
		if !out.sortRow(arcRow) {
			return nil, false, nil
		}
		for _, slab := range track.slabs {
			for _, root := range slab.root {
				radius := revolveLoftFacesHeelBoxRadius(root.box)
				if radius == nil {
					return nil, false, nil
				}
				out.curveBound = max(out.curveBound,
					proofbound.RatFloatUp(lfMul(big.NewRat(2, 1), radius)))
			}
		}
		for _, root := range track.bracket {
			if root == nil {
				return nil, false, nil
			}
			radius := revolveLoftFacesHeelBoxRadius(root.box)
			if radius == nil {
				return nil, false, nil
			}
			out.curveBound = max(out.curveBound,
				proofbound.RatFloatUp(lfMul(big.NewRat(2, 1), radius)))
		}
		stationRows := make([][]int, 0, len(track.stations))
		for index, station := range track.stations {
			if side == 0 && index == len(track.stations)-1 {
				continue // The exact lower contact circle uses shared blank ring IDs.
			}
			row, placed := out.stationRow(station, proof.trim.heel)
			if !placed {
				return nil, false, nil
			}
			stationRows = append(stationRows, row)
		}
		ringRow := append([]int(nil), rings[side].occupied...)
		if !out.sortRow(ringRow) ||
			!out.ringEndsCurrent(ringRow, rings[side]) {
			return nil, false, nil
		}
		if side == 0 {
			out.rows[side] = append(out.rows[side], arcRow)
			out.rows[side] = append(out.rows[side], stationRows...)
			out.rows[side] = append(out.rows[side], ringRow)
		} else {
			out.rows[side] = append(out.rows[side], ringRow)
			out.rows[side] = append(out.rows[side], stationRows...)
			out.rows[side] = append(out.rows[side], arcRow)
		}
		for index := 1; index < len(out.rows[side]); index++ {
			if !out.stitchRows(out.rows[side][index-1], out.rows[side][index]) {
				return nil, false, nil
			}
		}
	}
	if len(out.mesh.tris) == 0 || len(out.mesh.src) != len(out.mesh.tris) ||
		out.chord >= 0.1 || !out.patchEdgesBounded() {
		return nil, false, nil
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx),
		out.mesh.verts, out.mesh.tris, len(out.mesh.tris), 1<<20); err != nil {
		return nil, false, err
	}
	return out, true, nil
}

func (p *revolveLoftFacesHeelRetainedPatches) add(point r3.Vec, bound float64) int {
	id := len(p.mesh.verts)
	p.mesh.verts = append(p.mesh.verts, point)
	p.bounds = append(p.bounds, bound)
	return id
}

func (p *revolveLoftFacesHeelRetainedPatches) stationRow(
	station revolveLoftFacesHeelTrackStation, heel *pointConeLimit) ([]int, bool) {
	if heel == nil || heel.apex == nil || heel.slope == nil ||
		station.point[0].X != station.point[1].X {
		return nil, false
	}
	x := lfF(station.point[0].X)
	radius := lfDiv(lfSub(heel.apex, x), heel.slope)
	if radius.Sign() <= 0 {
		return nil, false
	}
	center, rounding, ok := revolveLoftFacesHeldPoint(loftFaceRatVec{x, radius, lfZero()})
	if !ok || center.Y <= 0 || rounding >= 0.1 {
		return nil, false
	}
	row := []int{
		p.add(station.point[1], station.sourceBound[1]),
		p.add(center, rounding),
		p.add(station.point[0], station.sourceBound[0]),
	}
	return row, p.sortRow(row)
}

func (p *revolveLoftFacesHeelRetainedPatches) sortRow(row []int) bool {
	if len(row) < 2 {
		return false
	}
	sort.Slice(row, func(i, j int) bool {
		a, b := p.mesh.verts[row[i]], p.mesh.verts[row[j]]
		return lfMul(lfF(a.Z), lfF(b.Y)).Cmp(lfMul(lfF(b.Z), lfF(a.Y))) < 0
	})
	for index, id := range row {
		point := p.mesh.verts[id]
		if point.Y <= 0 || !proofbound.FiniteVec(point) ||
			p.bounds[id] < 0 || p.bounds[id] >= 0.1 {
			return false
		}
		if index > 0 {
			previous := p.mesh.verts[row[index-1]]
			if lfMul(lfF(previous.Z), lfF(point.Y)).Cmp(
				lfMul(lfF(point.Z), lfF(previous.Y))) >= 0 {
				return false
			}
		}
	}
	return true
}

func (p *revolveLoftFacesHeelRetainedPatches) ringEndsCurrent(
	row []int, ring *revolveLoftFacesShellCapRing) bool {
	if ring == nil || len(row) < 3 ||
		row[0] != ring.endpoints[0] || row[len(row)-1] != ring.endpoints[1] {
		return false
	}
	return ring.endpointProof[0].point == p.mesh.verts[row[0]] &&
		ring.endpointProof[1].point == p.mesh.verts[row[len(row)-1]]
}

func (p *revolveLoftFacesHeelRetainedPatches) stitchRows(a, b []int) bool {
	if len(a) < 2 || len(b) < 2 {
		return false
	}
	x0, x1 := p.mesh.verts[a[0]].X, p.mesh.verts[b[0]].X
	if x0 >= x1 {
		return false
	}
	for _, row := range [][]int{a, b} {
		for _, id := range row {
			if p.mesh.verts[id].X != p.mesh.verts[row[0]].X {
				return false
			}
		}
	}
	i, j := 0, 0
	for i+1 < len(a) || j+1 < len(b) {
		var tri [3]int
		switch {
		case i+1 >= len(a):
			tri = [3]int{a[i], b[j], b[j+1]}
			j++
		case j+1 >= len(b):
			tri = [3]int{a[i], b[j], a[i+1]}
			i++
		case (i+1)*(len(b)-1) <= (j+1)*(len(a)-1):
			tri = [3]int{a[i], b[j], a[i+1]}
			i++
		default:
			tri = [3]int{a[i], b[j], b[j+1]}
			j++
		}
		if !p.chargeTriangle(tri) {
			return false
		}
		p.mesh.add(tri, 0)
	}
	return true
}

func (p *revolveLoftFacesHeelRetainedPatches) chargeTriangle(tri [3]int) bool {
	if p.heel == nil || p.heel.apex == nil || p.heel.slope == nil ||
		p.heel.slope.Sign() <= 0 {
		return false
	}
	var qLo, qHi, radius *big.Rat
	sourceBound := 0.0
	for _, id := range tri {
		point := p.mesh.verts[id]
		bound := lfF(p.bounds[id])
		y := loftFaceInterval{lfSub(lfF(point.Y), bound), lfAdd(lfF(point.Y), bound)}
		z := loftFaceInterval{lfSub(lfF(point.Z), bound), lfAdd(lfF(point.Z), bound)}
		if y.lo.Sign() <= 0 {
			return false
		}
		for _, yy := range []*big.Rat{y.lo, y.hi} {
			for _, zz := range []*big.Rat{z.lo, z.hi} {
				q := lfDiv(zz, yy)
				if qLo == nil || q.Cmp(qLo) < 0 {
					qLo = q
				}
				if qHi == nil || q.Cmp(qHi) > 0 {
					qHi = q
				}
			}
		}
		r := lfDiv(lfSub(p.heel.apex, lfSub(lfF(point.X), bound)), p.heel.slope)
		if r.Sign() <= 0 {
			return false
		}
		if radius == nil || r.Cmp(radius) > 0 {
			radius = r
		}
		sourceBound = max(sourceBound, p.bounds[id])
	}
	// atan is 1-Lipschitz, so the exact slope spread bounds angular spread.
	// The cone is ruled along X; its radial chord sagitta is R*angle²/8.
	spread := lfSub(qHi, qLo)
	chord := proofbound.RatFloatUp(lfDiv(lfMul(radius, lfMul(spread, spread)),
		big.NewRat(8, 1)))
	combined := proofbound.AbsSumUpper(chord, sourceBound, p.curveBound)
	if proofbound.IsNonFinite(chord) || proofbound.IsNonFinite(combined) ||
		combined >= 0.1 {
		return false
	}
	p.chord = max(p.chord, chord)
	p.patchBound = max(p.patchBound, combined)
	return true
}

func (p *revolveLoftFacesHeelRetainedPatches) patchEdgesBounded() bool {
	uses := make(map[[2]int]int)
	for _, tri := range p.mesh.tris {
		for side := range tri {
			a, b := tri[side], tri[(side+1)%3]
			if a == b || a < 0 || b < 0 || a >= len(p.mesh.verts) || b >= len(p.mesh.verts) {
				return false
			}
			key := [2]int{min(a, b), max(a, b)}
			uses[key]++
			if uses[key] > 2 {
				return false
			}
		}
	}
	for side, rows := range p.rows {
		if len(rows) < 3 {
			return false
		}
		ring := rows[0]
		if side == 0 {
			ring = rows[len(rows)-1]
		}
		for index := 1; index < len(ring); index++ {
			key := [2]int{min(ring[index-1], ring[index]),
				max(ring[index-1], ring[index])}
			if uses[key] != 1 {
				return false
			}
		}
	}
	return true
}
