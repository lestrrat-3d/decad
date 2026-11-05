package parts

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/explodeplan"
	"github.com/lestrrat-3d/decad/internal/jointinfer"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// bodyFaces is what the matching reads from one body: its cylinder and plane
// faces as plain records, with the faces they came from.
type bodyFaces struct {
	cyls       []jointinfer.Cylinder
	cylFaces   []*decad.Face
	planes     []jointinfer.Plane
	planeFaces []*decad.Face
	planeTris  [][][3]r3.Vec     // parallel to planes: the mesh triangles of each plane face
	unread     []explodeplan.Box // vertex boxes of the faces whose tag is neither a plane nor a cylinder
	unmeshed   bool              // the body would not tessellate, so no face could be read
	box        decad.Box
	meshTol    float64  // the chord tolerance the faces were read at, millimetres
	meshBound  float64  // that mesh's bound, millimetres
	verts      []r3.Vec // the mesh vertices, for the witness search
	verified   bool     // the mesh was built at VerifyAll
}

// faceTolerances are the tessellation tolerances tried in turn: the mesh is
// read only for which face each vertex belongs to, and a mesh Boolean result
// refuses a tolerance finer than its own bound.
var faceTolerances = []float64{0.05, 0.1, 0.2, 0.5, 5}

// readFaces reads the cylinder and plane faces of b. A face whose Surface is
// neither a Plane nor a Cylinder is counted and never asked for a normal.
func readFaces(ctx context.Context, b *decad.Body, verified bool) (*bodyFaces, error) {
	box, err := b.Bounds()
	if err != nil {
		return nil, err
	}
	bf := &bodyFaces{box: box}
	mesh, tol, ver, err := meshOf(ctx, b, verified)
	if err != nil {
		if errors.Is(err, decad.ErrUnsupported) {
			bf.unmeshed = true
			return bf, nil
		}
		return nil, err
	}
	bf.meshTol, bf.meshBound, bf.verified = tol, mesh.Bound().Base(), ver
	verts, tris, src := mesh.Vertices(), mesh.Triangles(), mesh.SourceFaces()
	bf.verts = verts
	perFace := map[*decad.Face][]int{}
	for ti, t := range tris {
		perFace[src[ti]] = append(perFace[src[ti]], t[0], t[1], t[2])
	}
	for _, f := range b.Faces() {
		idx := perFace[f]
		if len(idx) == 0 {
			continue
		}
		switch s := f.Surface().(type) {
		case decad.Plane:
			bf.addPlane(f, verts, idx)
		case decad.Cylinder:
			bf.addCylinder(f, s, verts, idx)
		default:
			bf.markUnread(verts, idx)
		}
	}
	return bf, nil
}

// meshOf tessellates b at the first of faceTolerances it accepts, and says the
// tolerance it settled on and whether the mesh is VerifyAll. With verified, every
// rung is tried at VerifyAll first, and only a body that refuses all of them is
// read at VerifyNone.
func meshOf(ctx context.Context, b *decad.Body, verified bool) (*decad.Mesh, float64, bool, error) {
	levels := []decad.Verification{decad.VerifyNone}
	if verified {
		levels = []decad.Verification{decad.VerifyAll, decad.VerifyNone}
	}
	var err error
	for _, level := range levels {
		for _, tol := range faceTolerances {
			var mesh *decad.Mesh
			mesh, err = b.Tessellate(ctx, units.Millimeters(tol), decad.WithVerification(level))
			if err == nil {
				return mesh, tol, level == decad.VerifyAll, nil
			}
			if !errors.Is(err, decad.ErrUnsupported) {
				return nil, 0, false, err
			}
		}
	}
	return nil, 0, false, err
}

func (bf *bodyFaces) addPlane(f *decad.Face, verts []r3.Vec, idx []int) {
	v0 := verts[idx[0]]
	n, err := f.NormalAt(v0)
	if err != nil {
		bf.markUnread(verts, idx)
		return
	}
	u, ok := n.Value.Normalize()
	if !ok {
		bf.markUnread(verts, idx)
		return
	}
	lo, hi := boxOf(verts, idx)
	tris := make([][3]r3.Vec, 0, len(idx)/3)
	for k := 0; k+2 < len(idx); k += 3 {
		tris = append(tris, [3]r3.Vec{verts[idx[k]], verts[idx[k+1]], verts[idx[k+2]]})
	}
	bf.planeTris = append(bf.planeTris, tris)
	bf.planes = append(bf.planes, jointinfer.Plane{Normal: u, Offset: u.Dot(v0), Min: lo, Max: hi,
		Bound: n.Bound.Base()})
	bf.planeFaces = append(bf.planeFaces, f)
}

func (bf *bodyFaces) addCylinder(f *decad.Face, s decad.Cylinder, verts []r3.Vec, idx []int) {
	axis, ok := s.Axis.Normalize()
	if !ok {
		bf.markUnread(verts, idx)
		return
	}
	v0 := verts[idx[0]]
	n, err := f.NormalAt(v0)
	if err != nil {
		bf.markUnread(verts, idx)
		return
	}
	radial := v0.Sub(s.Origin)
	radial = radial.Sub(axis.Scale(radial.Dot(axis)))
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, i := range idx {
		a := verts[i].Sub(s.Origin).Dot(axis)
		lo, hi = math.Min(lo, a), math.Max(hi, a)
	}
	bf.cyls = append(bf.cyls, jointinfer.Cylinder{Origin: s.Origin, Axis: axis, Radius: s.Radius.Base(),
		Convex: n.Value.Dot(radial) > 0, Lo: lo, Hi: hi, Bound: n.Bound.Base()})
	bf.cylFaces = append(bf.cylFaces, f)
}

// markUnread records a face the matching cannot read, by its vertex box.
func (bf *bodyFaces) markUnread(verts []r3.Vec, idx []int) {
	lo, hi := boxOf(verts, idx)
	bf.unread = append(bf.unread, explodeplan.Box{Min: lo, Max: hi})
}

func boxOf(verts []r3.Vec, idx []int) (r3.Vec, r3.Vec) {
	lo := r3.Vec{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	hi := r3.Vec{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
	for _, i := range idx {
		v := verts[i]
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	return lo, hi
}

// faceCache reads each body once per query.
type faceCache struct {
	byID     map[*decad.Body]*bodyFaces
	verified bool
}

// newFaceCache returns a cache that meshes bodies at VerifyNone, or at VerifyAll
// first when verified.
func newFaceCache(verified bool) *faceCache {
	return &faceCache{byID: map[*decad.Body]*bodyFaces{}, verified: verified}
}

func (c *faceCache) get(ctx context.Context, b *decad.Body) (*bodyFaces, error) {
	if bf, ok := c.byID[b]; ok {
		return bf, nil
	}
	bf, err := readFaces(ctx, b, c.verified)
	if err != nil {
		return nil, err
	}
	c.byID[b] = bf
	return bf, nil
}
