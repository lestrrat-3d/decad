package decad

import (
	"context"
	"fmt"
	"math"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

const (
	obliqueBossAngularCells = 128
	obliqueBossBlendCells   = 16
	obliqueBossAuditScans   = 16_000_000
)

type obliqueBossSample struct {
	u      float64
	corner int
}

type obliqueBossMesh struct {
	verts  []r3.Vec
	tris   [][3]int
	src    []int
	groups []facetGroup
}

func (m *obliqueBossMesh) point(p r3.Vec) int {
	i := len(m.verts)
	m.verts = append(m.verts, p)
	return i
}

func (m *obliqueBossMesh) triangle(a, b, c, source int) {
	m.tris = append(m.tris, [3]int{a, b, c})
	m.src = append(m.src, source)
}

func (m *obliqueBossMesh) quad(a, b, c, d, source int) {
	m.triangle(a, b, c, source)
	m.triangle(a, c, d, source)
}

func obliqueBossFaceGroup(body *Body, role string) (facetGroup, bool) {
	for _, face := range body.Faces() {
		for _, origin := range face.originRefs() {
			if origin.Role == role && origin.Producer == body.n.Origin.Producer {
				return facetGroup{Origins: face.originRefs(), Planar: face.isPlanar()}, true
			}
		}
	}
	return facetGroup{}, false
}

func obliqueBossHasOrigin(face *topo.Face, group facetGroup) bool {
	for _, origin := range face.Origins {
		if slices.Contains(group.Origins, origin) {
			return true
		}
	}
	return false
}

func obliqueBossPlateWallRoles(rec *obliqueBossRecord) ([4]string, bool) {
	pp, ok := rec.Plate.Payload.(prismPayload)
	if !ok || len(pp.Profile.Outer.Segments) != 4 {
		return [4]string{}, false
	}
	var roles [4]string // x-low, x-high, y-low, y-high
	for i, segment := range pp.Profile.Outer.Segments {
		line, ok := segment.(lineSeg)
		if !ok {
			return [4]string{}, false
		}
		role := fmt.Sprintf("side(0,%d)", i)
		switch {
		case line.Start.U == rec.Rect.U0 && line.End.U == rec.Rect.U0:
			roles[0] = role
		case line.Start.U == rec.Rect.U1 && line.End.U == rec.Rect.U1:
			roles[1] = role
		case line.Start.V == rec.Rect.V0 && line.End.V == rec.Rect.V0:
			roles[2] = role
		case line.Start.V == rec.Rect.V1 && line.End.V == rec.Rect.V1:
			roles[3] = role
		default:
			return [4]string{}, false
		}
	}
	for _, role := range roles {
		if role == "" {
			return [4]string{}, false
		}
	}
	return roles, true
}

// filletObliqueBoss uses the joined body's edge only to authenticate the
// selector. Its source record supplies every geometric surface.
func (b *Body) filletObliqueBoss(ctx context.Context, edges []*Edge,
	rec *obliqueBossRecord, radius, radiusDelta float64) (*Body, error) {
	if len(edges) != 1 || edges[0].n.Start != edges[0].n.End {
		return nil, fmt.Errorf("%w: an oblique boss fillet needs its one complete junction edge", ErrUnsupported)
	}
	if _, ok := edges[0].n.Curve.(surfacegeom.FacetedCurve); !ok {
		return nil, fmt.Errorf("%w: the selected oblique junction has no faceted edge witness", ErrUnsupported)
	}
	plateTop, topOK := obliqueBossFaceGroup(bodyHandle(rec.Plate), provenance.RoleCapEnd)
	bossSide, sideOK := obliqueBossFaceGroup(bodyHandle(rec.Boss), prismrecord.OuterFirstSideRole)
	if !topOK || !sideOK || len(edges[0].n.Faces) != 2 {
		return nil, fmt.Errorf("%w: the selected edge is not the complete plate-to-cylinder junction", ErrUnsupported)
	}
	matched := (obliqueBossHasOrigin(edges[0].n.Faces[0], plateTop) &&
		obliqueBossHasOrigin(edges[0].n.Faces[1], bossSide)) ||
		(obliqueBossHasOrigin(edges[0].n.Faces[1], plateTop) &&
			obliqueBossHasOrigin(edges[0].n.Faces[0], bossSide))
	if !matched {
		return nil, fmt.Errorf("%w: the selected edge is not the complete plate-to-cylinder junction", ErrUnsupported)
	}
	if radiusDelta != 0 || !(radius > 0) ||
		proofbound.IsNonFinite(radius) || radius >= rec.Radius/4 {
		return nil, fmt.Errorf("%w: the oblique junction fillet radius exceeds its proven clearance", ErrUnsupported)
	}
	margin := proofbound.DivUpper(proofbound.AbsSumUpper(rec.Radius, radius), 0.93)
	center := rec.Base.Add(rec.Axis.Scale((rec.Top + radius - rec.Base.Z) / rec.Axis.Z))
	if !(center.X-margin-rec.SourceBound > rec.Rect.U0 &&
		center.X+margin+rec.SourceBound < rec.Rect.U1 &&
		center.Y-margin-rec.SourceBound > rec.Rect.V0 &&
		center.Y+margin+rec.SourceBound < rec.Rect.V1 &&
		rec.Base.Z+rec.Length*rec.Axis.Z-rec.Radius*math.Hypot(rec.Axis.X, rec.Axis.Y)-rec.SourceBound >
			rec.Top+radius*(1+math.Hypot(rec.Axis.X, rec.Axis.Y))) {
		return nil, fmt.Errorf("%w: the oblique junction fillet reaches a plate side or the boss far cap", ErrUnsupported)
	}
	mesh, err := buildObliqueBossFilletMesh(ctx, rec, radius)
	if err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx),
		mesh.verts, mesh.tris, loftmesh.MaxLoftAuditTriangles, obliqueBossAuditScans); err != nil {
		return nil, err
	}
	bound, volumeGap, areaSlack, err := proveObliqueBossFillet(ctx, rec, radius, mesh)
	if err != nil {
		return nil, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, mesh.verts, mesh.tris)
	if err != nil {
		return nil, err
	}
	volume, heldVolume, err := meshVolumeMeasurement(ctx, audit, volumeGap)
	if err != nil {
		return nil, err
	}
	if heldVolume.Cmp(proofarith.FloatRat(volumeGap)) <= 0 {
		return nil, fmt.Errorf("%w: the oblique fillet has no proven positive occupied volume", ErrUnsupported)
	}
	dPair := obliqueBossDiameterBound(rec, bound)
	if proofbound.IsNonFinite(dPair) {
		return nil, fmt.Errorf("%w: the oblique boss has no finite diameter proof", ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	ref := b.n.Doc.NextProducer
	mesh.groups[8].Origins[0].Producer = ref
	result, err := bodyResult(facetedrecord.BuildWithProof(ctx, envOf(documentNode(documentHandle(b.n.Doc)), ref), facetedPayload{
		Verts: mesh.verts, Tris: mesh.tris, Src: mesh.src, Groups: mesh.groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: dPair, ContactAudited: true, Xform: r3.Identity(),
	}, audit, measurementToInternal(volume), heldVolume))
	if err != nil {
		return nil, err
	}
	return commitModifyResult(ctx, b, result)
}

func obliqueBossSamples(rec *obliqueBossRecord, radius float64) []obliqueBossSample {
	samples := make([]obliqueBossSample, 0, obliqueBossAngularCells+4)
	for i := range obliqueBossAngularCells {
		samples = append(samples, obliqueBossSample{u: 2 * math.Pi * float64(i) / obliqueBossAngularCells, corner: -1})
	}
	center := rec.Base.Add(rec.Axis.Scale((rec.Top + radius - rec.Base.Z) / rec.Axis.Z))
	corners := [4]r3.Vec{
		{X: rec.Rect.U1, Y: rec.Rect.V0},
		{X: rec.Rect.U1, Y: rec.Rect.V1},
		{X: rec.Rect.U0, Y: rec.Rect.V1},
		{X: rec.Rect.U0, Y: rec.Rect.V0},
	}
	sinTilt := math.Hypot(rec.Axis.X, rec.Axis.Y)
	hx, hy := rec.Axis.X/sinTilt, rec.Axis.Y/sinTilt
	for i, corner := range corners {
		dx, dy := corner.X-center.X, corner.Y-center.Y
		u := math.Atan2(-hy*dx+hx*dy, rec.Axis.Z*(hx*dx+hy*dy))
		if u < 0 {
			u += 2 * math.Pi
		}
		samples = append(samples, obliqueBossSample{u: u, corner: i})
	}
	sort.Slice(samples, func(i, j int) bool { return samples[i].u < samples[j].u })
	// A square corner can coincide with a regular angular station. Keep the
	// corner so the outer rim changes wall only at an explicit mesh vertex.
	const sameStation = 1e-12
	unique := samples[:0]
	for _, sample := range samples {
		if len(unique) != 0 && sample.u-unique[len(unique)-1].u < sameStation {
			if sample.corner >= 0 {
				unique[len(unique)-1] = sample
			}
			continue
		}
		unique = append(unique, sample)
	}
	if len(unique) > 1 && unique[0].u+2*math.Pi-unique[len(unique)-1].u < sameStation {
		if unique[len(unique)-1].corner >= 0 {
			unique = unique[1:]
		} else {
			unique = unique[:len(unique)-1]
		}
	}
	return unique
}

// obliqueBossPlaneRay follows the plate-plane trace of the circle's
// perpendicular section. The same ray sets the rim vertex and corner angle.
func obliqueBossPlaneRay(rec *obliqueBossRecord, u float64) (float64, float64) {
	sn, cs := math.Sincos(u)
	d := rec.Ex.Scale(cs).Add(rec.Ey.Scale(sn))
	return d.X - rec.Axis.X*d.Z/rec.Axis.Z,
		d.Y - rec.Axis.Y*d.Z/rec.Axis.Z
}

func obliqueBossOuterPoint(rec *obliqueBossRecord, center r3.Vec,
	u float64, corner int) r3.Vec {
	switch corner {
	case 0:
		return r3.Vec{X: rec.Rect.U1, Y: rec.Rect.V0, Z: rec.Top}
	case 1:
		return r3.Vec{X: rec.Rect.U1, Y: rec.Rect.V1, Z: rec.Top}
	case 2:
		return r3.Vec{X: rec.Rect.U0, Y: rec.Rect.V1, Z: rec.Top}
	case 3:
		return r3.Vec{X: rec.Rect.U0, Y: rec.Rect.V0, Z: rec.Top}
	}
	dx, dy := obliqueBossPlaneRay(rec, u)
	best := math.Inf(1)
	side := -1
	for i, x := range []float64{rec.Rect.U0, rec.Rect.U1} {
		if dx == 0 {
			continue
		}
		t := (x - center.X) / dx
		if t > 0 && t < best {
			best, side = t, i
		}
	}
	for i, y := range []float64{rec.Rect.V0, rec.Rect.V1} {
		if dy == 0 {
			continue
		}
		t := (y - center.Y) / dy
		if t > 0 && t < best {
			best, side = t, i+2
		}
	}
	p := r3.Vec{X: center.X + best*dx, Y: center.Y + best*dy, Z: rec.Top}
	switch side {
	case 0:
		p.X = rec.Rect.U0
	case 1:
		p.X = rec.Rect.U1
	case 2:
		p.Y = rec.Rect.V0
	case 3:
		p.Y = rec.Rect.V1
	}
	return p
}

func buildObliqueBossFilletMesh(ctx context.Context,
	rec *obliqueBossRecord, radius float64) (*obliqueBossMesh, error) {
	m := &obliqueBossMesh{}
	wallRoles, ok := obliqueBossPlateWallRoles(rec)
	if !ok {
		return nil, fmt.Errorf("%w: the plate walls have no source role map", ErrUnsupported)
	}
	roles := []struct {
		body *Body
		role string
	}{
		{bodyHandle(rec.Plate), provenance.RoleCapStart},
		{bodyHandle(rec.Plate), wallRoles[0]}, {bodyHandle(rec.Plate), wallRoles[1]},
		{bodyHandle(rec.Plate), wallRoles[2]}, {bodyHandle(rec.Plate), wallRoles[3]},
		{bodyHandle(rec.Plate), provenance.RoleCapEnd},
		{bodyHandle(rec.Boss), prismrecord.OuterFirstSideRole}, {bodyHandle(rec.Boss), provenance.RoleCapEnd},
	}
	for _, role := range roles {
		group, ok := obliqueBossFaceGroup(role.body, role.role)
		if !ok {
			return nil, fmt.Errorf("%w: a retained oblique boss source has no face %q", ErrUnsupported, role.role)
		}
		m.groups = append(m.groups, group)
	}
	m.groups = append(m.groups, facetGroup{Origins: []provenance.Ref{{Role: "junctionFillet"}}})
	samples := obliqueBossSamples(rec, radius)
	n := len(samples)
	outer, lower, plane, side, far := make([]int, n), make([]int, n),
		make([]int, n), make([]int, n), make([]int, n)
	band := make([][]int, obliqueBossBlendCells+1)
	for j := range band {
		band[j] = make([]int, n)
	}
	center := rec.Base.Add(rec.Axis.Scale((rec.Top + radius - rec.Base.Z) / rec.Axis.Z))
	for i, sample := range samples {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		sn, cs := math.Sincos(sample.u)
		d := rec.Ex.Scale(cs).Add(rec.Ey.Scale(sn))
		t := (rec.Top + radius - rec.Base.Z - (rec.Radius+radius)*d.Z) / rec.Axis.Z
		c := rec.Base.Add(d.Scale(rec.Radius + radius)).Add(rec.Axis.Scale(t))
		p := c.Sub(r3.Vec{Z: radius})
		p.Z = rec.Top
		s := c.Sub(d.Scale(radius))
		u := rec.Base.Add(d.Scale(rec.Radius)).Add(rec.Axis.Scale(rec.Length))
		o := obliqueBossOuterPoint(rec, center, sample.u, sample.corner)
		if !proofbound.FiniteVec(o) || o.X < rec.Rect.U0 || o.X > rec.Rect.U1 ||
			o.Y < rec.Rect.V0 || o.Y > rec.Rect.V1 {
			return nil, fmt.Errorf("%w: an oblique plate rim leaves its source rectangle", ErrUnsupported)
		}
		outer[i], lower[i], plane[i], side[i], far[i] =
			m.point(o), m.point(r3.Vec{X: o.X, Y: o.Y, Z: rec.Bottom}),
			m.point(p), m.point(s), m.point(u)
		band[0][i], band[obliqueBossBlendCells][i] = plane[i], side[i]
		for j := 1; j < obliqueBossBlendCells; j++ {
			v := float64(j) / obliqueBossBlendCells
			q := r3.Vec{Z: 1 - v}.Add(d.Scale(v))
			band[j][i] = m.point(c.Sub(q.Scale(radius / q.Len())))
		}
	}
	bottomCenter := m.point(r3.Vec{X: (rec.Rect.U0 + rec.Rect.U1) / 2,
		Y: (rec.Rect.V0 + rec.Rect.V1) / 2, Z: rec.Bottom})
	farCenter := m.point(rec.Base.Add(rec.Axis.Scale(rec.Length)))
	for i := range samples {
		j := (i + 1) % n
		a, b := m.verts[outer[i]], m.verts[outer[j]]
		wall := -1
		switch {
		case a.X == rec.Rect.U0 && b.X == rec.Rect.U0:
			wall = 1
		case a.X == rec.Rect.U1 && b.X == rec.Rect.U1:
			wall = 2
		case a.Y == rec.Rect.V0 && b.Y == rec.Rect.V0:
			wall = 3
		case a.Y == rec.Rect.V1 && b.Y == rec.Rect.V1:
			wall = 4
		}
		if wall < 0 {
			return nil, fmt.Errorf("%w: an oblique plate rim jumps across a corner", ErrUnsupported)
		}
		m.triangle(bottomCenter, lower[j], lower[i], 0)
		m.quad(lower[i], lower[j], outer[j], outer[i], wall)
		m.quad(outer[i], outer[j], plane[j], plane[i], 5)
		m.quad(side[i], side[j], far[j], far[i], 6)
		m.triangle(farCenter, far[i], far[j], 7)
		for k := range obliqueBossBlendCells {
			m.quad(band[k][i], band[k][j], band[k+1][j], band[k+1][i], 8)
		}
	}
	return m, nil
}
