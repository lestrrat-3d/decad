package obliqueboss

import (
	"context"
	"fmt"
	"math"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

const (
	angularCells = 128
	blendCells   = 16
	// AuditScans is the crossing-audit scan budget of one fillet mesh.
	AuditScans = 16_000_000
)

type station struct {
	u      float64
	corner int
}

// Mesh is the held fillet mesh: its vertices, triangles, each triangle's
// source group index into Groups, and the source face groups.
type Mesh struct {
	Verts  []r3.Vec
	Tris   [][3]int
	Src    []int
	Groups []facetedrecord.Group
}

func (m *Mesh) point(p r3.Vec) int {
	i := len(m.Verts)
	m.Verts = append(m.Verts, p)
	return i
}

func (m *Mesh) triangle(a, b, c, source int) {
	m.Tris = append(m.Tris, [3]int{a, b, c})
	m.Src = append(m.Src, source)
}

func (m *Mesh) quad(a, b, c, d, source int) {
	m.triangle(a, b, c, source)
	m.triangle(a, c, d, source)
}

// FaceGroup returns the provenance group of the first face of body that
// carries role under the body's own producer.
func FaceGroup(body *topo.Body, role string) (facetedrecord.Group, bool) {
	for _, face := range topobuild.Faces(body) {
		for _, origin := range append([]provenance.Ref(nil), face.Origins...) {
			if origin.Role == role && origin.Producer == body.Origin.Producer {
				return facetedrecord.Group{Origins: append([]provenance.Ref(nil), face.Origins...),
					Planar: face.Surface.Kind() == surfacegeom.KindPlane || face.HeldPlanar}, true
			}
		}
	}
	return facetedrecord.Group{}, false
}

// HasOrigin reports whether face carries any origin of group.
func HasOrigin(face *topo.Face, group facetedrecord.Group) bool {
	for _, origin := range face.Origins {
		if slices.Contains(group.Origins, origin) {
			return true
		}
	}
	return false
}

func plateWallRoles(rec *facetedrecord.ObliqueBossRecord) ([4]string, bool) {
	pp, ok := rec.Plate.Payload.(prismrecord.Payload)
	if !ok || len(pp.Profile.Outer.Segments) != 4 {
		return [4]string{}, false
	}
	var roles [4]string // x-low, x-high, y-low, y-high
	for i, segment := range pp.Profile.Outer.Segments {
		line, ok := segment.(sectionrecord.LineSeg)
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

func samplePlan(rec *facetedrecord.ObliqueBossRecord, radius float64) []station {
	samples := make([]station, 0, angularCells+4)
	for i := range angularCells {
		samples = append(samples, station{u: 2 * math.Pi * float64(i) / angularCells, corner: -1})
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
		samples = append(samples, station{u: u, corner: i})
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

// planeRay follows the plate-plane trace of the circle's
// perpendicular section. The same ray sets the rim vertex and corner angle.
func planeRay(rec *facetedrecord.ObliqueBossRecord, u float64) (float64, float64) {
	sn, cs := math.Sincos(u)
	d := rec.Ex.Scale(cs).Add(rec.Ey.Scale(sn))
	return d.X - rec.Axis.X*d.Z/rec.Axis.Z,
		d.Y - rec.Axis.Y*d.Z/rec.Axis.Z
}

func outerPoint(rec *facetedrecord.ObliqueBossRecord, center r3.Vec,
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
	dx, dy := planeRay(rec, u)
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

// BuildFilletMesh builds the held mesh of the fillet of radius between the
// plate's top and the boss cylinder, polling ctx every 32 stations.
func BuildFilletMesh(ctx context.Context,
	rec *facetedrecord.ObliqueBossRecord, radius float64) (*Mesh, error) {
	m := &Mesh{}
	wallRoles, ok := plateWallRoles(rec)
	if !ok {
		return nil, fmt.Errorf("%w: the plate walls have no source role map", decaderr.ErrUnsupported)
	}
	roles := []struct {
		body *topo.Body
		role string
	}{
		{rec.Plate, provenance.RoleCapStart},
		{rec.Plate, wallRoles[0]}, {rec.Plate, wallRoles[1]},
		{rec.Plate, wallRoles[2]}, {rec.Plate, wallRoles[3]},
		{rec.Plate, provenance.RoleCapEnd},
		{rec.Boss, prismrecord.OuterFirstSideRole}, {rec.Boss, provenance.RoleCapEnd},
	}
	for _, role := range roles {
		group, ok := FaceGroup(role.body, role.role)
		if !ok {
			return nil, fmt.Errorf("%w: a retained oblique boss source has no face %q", decaderr.ErrUnsupported, role.role)
		}
		m.Groups = append(m.Groups, group)
	}
	m.Groups = append(m.Groups, facetedrecord.Group{Origins: []provenance.Ref{{Role: "junctionFillet"}}})
	samples := samplePlan(rec, radius)
	n := len(samples)
	outer, lower, plane, side, far := make([]int, n), make([]int, n),
		make([]int, n), make([]int, n), make([]int, n)
	band := make([][]int, blendCells+1)
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
		o := outerPoint(rec, center, sample.u, sample.corner)
		if !proofbound.FiniteVec(o) || o.X < rec.Rect.U0 || o.X > rec.Rect.U1 ||
			o.Y < rec.Rect.V0 || o.Y > rec.Rect.V1 {
			return nil, fmt.Errorf("%w: an oblique plate rim leaves its source rectangle", decaderr.ErrUnsupported)
		}
		outer[i], lower[i], plane[i], side[i], far[i] =
			m.point(o), m.point(r3.Vec{X: o.X, Y: o.Y, Z: rec.Bottom}),
			m.point(p), m.point(s), m.point(u)
		band[0][i], band[blendCells][i] = plane[i], side[i]
		for j := 1; j < blendCells; j++ {
			v := float64(j) / blendCells
			q := r3.Vec{Z: 1 - v}.Add(d.Scale(v))
			band[j][i] = m.point(c.Sub(q.Scale(radius / q.Len())))
		}
	}
	bottomCenter := m.point(r3.Vec{X: (rec.Rect.U0 + rec.Rect.U1) / 2,
		Y: (rec.Rect.V0 + rec.Rect.V1) / 2, Z: rec.Bottom})
	farCenter := m.point(rec.Base.Add(rec.Axis.Scale(rec.Length)))
	for i := range samples {
		j := (i + 1) % n
		a, b := m.Verts[outer[i]], m.Verts[outer[j]]
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
			return nil, fmt.Errorf("%w: an oblique plate rim jumps across a corner", decaderr.ErrUnsupported)
		}
		m.triangle(bottomCenter, lower[j], lower[i], 0)
		m.quad(lower[i], lower[j], outer[j], outer[i], wall)
		m.quad(outer[i], outer[j], plane[j], plane[i], 5)
		m.quad(side[i], side[j], far[j], far[i], 6)
		m.triangle(farCenter, far[i], far[j], 7)
		for k := range blendCells {
			m.quad(band[k][i], band[k][j], band[k+1][j], band[k+1][i], 8)
		}
	}
	return m, nil
}
