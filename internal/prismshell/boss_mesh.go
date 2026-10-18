package prismshell

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// BossMeshInput contains the authenticated source data needed to build and
// prove the inward cup mesh. The caller retains source authentication.
type BossMeshInput struct {
	Verts                                        []r3.Vec
	VertexBound                                  []float64
	Groups                                       []facetedrecord.Group
	Tris                                         [][3]int
	Src                                          []int
	Boss                                         prismrecord.Payload
	PlateRect, BossRect                          BossRect
	PlateBottom, InterfaceZ, BossBottom, BossTop float64
	UnitLo, UnitHi                               float64
	Outer                                        [4]int
	TopGroup                                     int
	Stations                                     int
}

// BossMesh is a transient mesh for the contact and volume audits.
type BossMesh struct {
	Verts                   []r3.Vec
	Tris                    [][3]int
	Src                     []int
	Groups                  []facetedrecord.Group
	VertexBound             []float64
	MaxVertexError          float64
	RootStart, BossPointEnd int
}

// PrismUnionBossShellCapOwners matches the removed cap's two triangles to
// its four recorded boss corners and rejects any shared or misplaced source
// group. The caller has already authenticated the live opening face.
func PrismUnionBossShellCapOwners(fp facetedrecord.Payload, openingIdx int,
	openingOrigin provenance.Ref, corners [4]r3.Vec) ([4]int, int, bool) {
	var outer [4]int
	var topVerts map[int]struct{}
	var topTris [][3]int
	topGroup := 0
	for fi, faceIdx := range fp.FaceOf {
		if faceIdx != openingIdx {
			continue
		}
		if topVerts == nil {
			topVerts = map[int]struct{}{}
			topGroup = fp.Src[fi]
		}
		if fp.Src[fi] != topGroup {
			return outer, topGroup, false
		}
		topTris = append(topTris, fp.Tris[fi])
		for _, vi := range fp.Tris[fi] {
			topVerts[vi] = struct{}{}
		}
	}
	if len(topVerts) != 4 || len(topTris) != 2 || topGroup < 0 || topGroup >= len(fp.Groups) ||
		len(fp.Groups[topGroup].Origins) != 1 || !fp.Groups[topGroup].Planar ||
		fp.Groups[topGroup].Origins[0] != openingOrigin {
		return outer, topGroup, false
	}
	for fi, group := range fp.Src {
		if group == topGroup && fp.FaceOf[fi] != openingIdx {
			return outer, topGroup, false
		}
	}
	for i, group := range fp.Groups {
		if i == topGroup {
			continue
		}
		if slices.Contains(group.Origins, openingOrigin) {
			return outer, topGroup, false
		}
	}
	for i, p := range corners {
		found := -1
		for vi := range topVerts {
			if fp.Verts[vi] == p {
				found = vi
				break
			}
		}
		if found < 0 {
			return outer, topGroup, false
		}
		outer[i] = found
	}
	if !PrismUnionBossShellCompleteCap(outer, topTris) {
		return outer, topGroup, false
	}
	return outer, topGroup, true
}

func PrismUnionBossShellMeshOf(ctx context.Context, ref provenance.Producer,
	input BossMeshInput, t float64) (BossMesh, error) {
	var out BossMesh
	out.Verts = append(out.Verts, input.Verts...)
	out.VertexBound = append(out.VertexBound, input.VertexBound...)
	out.Groups = append(out.Groups, input.Groups...)
	addGroup := func(role string, planar bool) int {
		i := len(out.Groups)
		out.Groups = append(out.Groups, facetedrecord.Group{Origins: []provenance.Ref{{Producer: ref, Role: role}}, Planar: planar})
		return i
	}
	// The source's top cap is replaced by the opening's narrow horizontal rim.
	for i, tri := range input.Tris {
		if input.Src[i] == input.TopGroup {
			continue
		}
		out.Tris = append(out.Tris, tri)
		out.Src = append(out.Src, input.Src[i])
	}
	add := func(v r3.Vec, beta float64) int {
		i := len(out.Verts)
		out.Verts = append(out.Verts, v)
		out.VertexBound = append(out.VertexBound, beta)
		out.MaxVertexError = max(out.MaxVertexError, beta)
		return i
	}
	tri := func(a, b, c, group int) {
		out.Tris = append(out.Tris, [3]int{a, b, c})
		out.Src = append(out.Src, group)
	}
	quad := func(bottom, top [4]int, groups [4]int) {
		for i := range 4 {
			j := (i + 1) % 4
			tri(bottom[i], top[i], top[j], groups[i])
			tri(bottom[i], top[j], bottom[j], groups[i])
		}
	}
	sideGroups := func(role string, planar bool) [4]int {
		var groups [4]int
		for i := range groups {
			groups[i] = addGroup(fmt.Sprintf("%s(%d)", role, i), planar)
		}
		return groups
	}
	point := func(corner int, qWorld, qError, zWorld, zError float64) (int, error) {
		uv := PrismUnionBossShellCorners(input.BossRect)[corner]
		sx, sy := 1.0, 1.0
		if corner == 1 || corner == 2 {
			sx = -1
		}
		if corner >= 2 {
			sy = -1
		}
		qLocal := proofbound.DivUpper(qWorld, input.UnitLo)
		qLoRat := new(big.Rat).Quo(new(big.Rat).Sub(
			proofarith.FloatRat(qWorld), proofarith.FloatRat(qError)), proofarith.FloatRat(input.UnitHi))
		qLo := proofbound.RatFloatDown(qLoRat)
		qBound := proofbound.ProvenUpRound(qLocal - qLo)
		localZ := zWorld - input.Boss.Frame.Origin().Z
		uLocal, vLocal := uv[0]+sx*qLocal, uv[1]+sy*qLocal
		uBound := proofbound.AbsSumUpper(qBound, proofbound.ExactSumRound(uLocal, uv[0], sx*qLocal))
		vBound := proofbound.AbsSumUpper(qBound, proofbound.ExactSumRound(vLocal, uv[1], sy*qLocal))
		zBound := proofbound.AbsSumUpper(zError,
			proofbound.ExactSumRound(localZ, zWorld, -input.Boss.Frame.Origin().Z))
		v := input.Boss.Point(uLocal, vLocal, localZ)
		beta := input.Boss.PointBound(
			proofbound.MeasuredScalar(uLocal, uBound),
			proofbound.MeasuredScalar(vLocal, vBound),
			proofbound.MeasuredScalar(localZ, zBound))
		if !proofbound.FiniteVec(v) || proofbound.IsNonFinite(beta) || math.Abs(v.Z-zWorld) > beta {
			return 0, fmt.Errorf("%w: boss shell station has no bounded horizontal lift", decaderr.ErrUnsupported)
		}
		if (zWorld == input.BossBottom || zWorld == input.InterfaceZ || zWorld == input.BossTop) &&
			v.Z != zWorld {
			return 0, fmt.Errorf("%w: boss shell rim or ledge moved off its source plane", decaderr.ErrUnsupported)
		}
		return add(v, beta), nil
	}
	out.RootStart = len(out.Verts)
	var upperTop, upperBase [4]int
	for i := range 4 {
		var err error
		upperTop[i], err = point(i, t, 0, input.BossTop, 0)
		if err != nil {
			return out, err
		}
		upperBase[i], err = point(i, t, 0, input.InterfaceZ, 0)
		if err != nil {
			return out, err
		}
	}
	for i := range 4 {
		j := (i + 1) % 4
		g := addGroup(fmt.Sprintf("shellRim(%d)", i), true)
		tri(input.Outer[i], input.Outer[j], upperTop[j], g)
		tri(input.Outer[i], upperTop[j], upperTop[i], g)
	}
	quad(upperBase, upperTop, sideGroups("shellUpperWall", true))
	prev := upperBase
	rootGroups := sideGroups("shellRootBand", false)
	for k := input.Stations - 1; k >= 0; k-- {
		w := big.NewRat(int64(k), int64(input.Stations))
		w2 := new(big.Rat).Mul(w, w)
		den := new(big.Rat).Add(big.NewRat(1, 1), w2)
		qRat := new(big.Rat).Quo(new(big.Rat).Mul(proofarith.FloatRat(t),
			new(big.Rat).Mul(big.NewRat(2, 1), w2)), den)
		sRat := new(big.Rat).Quo(new(big.Rat).Mul(proofarith.FloatRat(t),
			new(big.Rat).Mul(big.NewRat(2, 1), w)), den)
		zRat := new(big.Rat).Add(proofarith.FloatRat(input.BossBottom), sRat)
		q := proofbound.RatFloatUp(qRat)
		z, _ := zRat.Float64()
		qError := proofarith.RationalFloatError(qRat, q)
		zError := proofarith.RationalFloatError(zRat, z)
		if k == 0 {
			q = 0
			z = input.BossBottom
			qError, zError = 0, 0
		}
		var ring [4]int
		for i := range 4 {
			var err error
			ring[i], err = point(i, q, qError, z, zError)
			if err != nil {
				return out, err
			}
		}
		quad(ring, prev, rootGroups)
		prev = ring
	}
	root := prev
	out.BossPointEnd = len(out.Verts)
	plateInner := input.PlateRect.Inset(t)
	plateCorners := PrismUnionBossShellCorners(plateInner)
	var ledgeOuter, floor [4]int
	for i, uv := range plateCorners {
		ledgeOuter[i] = add(r3.Vec{X: uv[0], Y: uv[1], Z: input.BossBottom}, 0)
		floor[i] = add(r3.Vec{X: uv[0], Y: uv[1], Z: input.PlateBottom + t}, 0)
	}
	pts := make([]triangulation.Point2, len(out.Verts))
	for i, v := range out.Verts {
		pts[i] = triangulation.Point2{U: v.X, V: v.Y}
	}
	ledge, err := triangulation.Triangulate(ctx, pts, [][]int{
		{ledgeOuter[0], ledgeOuter[1], ledgeOuter[2], ledgeOuter[3]},
		{root[3], root[2], root[1], root[0]},
	})
	if err != nil {
		return out, fmt.Errorf("%w: boss shell ledge triangulation: %v", decaderr.ErrUnsupported, err)
	}
	ledgeGroup := addGroup("shellLedge", true)
	for _, tr := range ledge {
		tri(tr[0], tr[2], tr[1], ledgeGroup)
	}
	quad(floor, ledgeOuter, sideGroups("shellLowerWall", true))
	floorGroup := addGroup("shellFloor", true)
	tri(floor[0], floor[1], floor[2], floorGroup)
	tri(floor[0], floor[2], floor[3], floorGroup)
	return out, nil
}
