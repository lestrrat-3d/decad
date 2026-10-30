package retainedblend

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Unread names the first selected edge that no class of Table RR reads, and the
// condition that keeps it out, for the generic fillet and chamfer refusal to
// append. It reads face tags and the edge's face count only, never the held
// geometry, so it states nothing the tags do not. It returns "" when every
// selected edge is in some class: the refusal then came from a later gate.
func Unread(edges []*topo.Edge) string {
	for i, e := range edges {
		if e == nil {
			return fmt.Sprintf("no retained blend class reads selected edge[%d]: it is not an edge", i)
		}
		if looksLikeRim(e) || looksLikeSphereRim(e) || looksLikeLine(e) || looksLikeCylLine(e) ||
			looksLikeArc(e) || looksLikeCylCyl(e) || looksLikeSaddle(e) {
			continue
		}
		faces := make([]tagFace, len(e.Faces))
		for j, f := range e.Faces {
			faces[j] = tagFaceOf(f)
		}
		return fmt.Sprintf("no retained blend class reads selected edge[%d]: %s", i, unreadReason(faces))
	}
	return ""
}

// tagFace is what a face's tag says: plain data, no geometry.
type tagFace struct {
	kind        surfacegeom.SurfaceKind
	retained    bool   // the face carries a retained tag
	departs     bool   // the tag states a departure from the true surface
	notDenoted  bool   // a denoted patch the tag is not exactly
	missing     bool   // the edge names no face here
	cylinderDir r3.Vec // the axis of a cylinder tag
}

func tagFaceOf(f *topo.Face) tagFace {
	if f == nil {
		return tagFace{missing: true}
	}
	if f.Surface == nil {
		return tagFace{kind: surfacegeom.KindFaceted}
	}
	t := tagFace{kind: f.Surface.Kind(), retained: f.RetainedTag, departs: f.TagDeparture != 0}
	if f.RetainedTag && !t.departs {
		// Only the kinds a class reads against a revolve record are judged: a
		// Cone or Torus with a record is named by its kind instead.
		switch surf := f.Surface.(type) {
		case surfacegeom.Plane, surfacegeom.Cylinder:
			t.notDenoted = !tagIsRecord(f)
		case surfacegeom.Sphere:
			t.notDenoted = sphereIsRecord(f, surf) != nil
		}
	}
	if c, ok := f.Surface.(surfacegeom.Cylinder); ok {
		t.cylinderDir = c.Axis
	}
	return t
}

var kindNames = map[surfacegeom.SurfaceKind]string{
	surfacegeom.KindPlane:    "plane",
	surfacegeom.KindCylinder: "cylinder",
	surfacegeom.KindCone:     "cone",
	surfacegeom.KindSphere:   "sphere",
	surfacegeom.KindTorus:    "torus",
}

// unreadReason states why no class reads an edge between faces, from their tags.
func unreadReason(faces []tagFace) string {
	if len(faces) != 2 {
		if len(faces) == 1 {
			return "the edge borders 1 face, and a class reads an edge between exactly two"
		}
		return fmt.Sprintf("the edge borders %d faces, and a class reads an edge between exactly two", len(faces))
	}
	for _, f := range faces {
		switch {
		case f.missing:
			return "the edge names a face that is not there"
		case f.kind == surfacegeom.KindFaceted:
			return "a face with no retained surface (faceted)"
		case f.kind == surfacegeom.KindNURBS:
			return "a face with a free-form surface (NURBS), which has no retained analytic surface"
		case !f.retained:
			return "a face whose surface is not a retained tag"
		case f.departs:
			return "a face whose retained tag states a departure from the true surface (retained-edge-blend SR1)"
		case f.notDenoted:
			return "a face whose tag is not proven to be exactly the surface its revolve record denotes (retained-edge-blend SR1)"
		}
	}
	a, b := faces[0], faces[1]
	switch {
	case a.kind == surfacegeom.KindCylinder && b.kind == surfacegeom.KindCylinder &&
		a.cylinderDir.Cross(b.cylinderDir).Len() > parallelTol:
		return "two retained cylinders whose axes are not parallel: the blend is a canal surface " +
			"(retained-edge-blend SR1)"
	case a.kind == surfacegeom.KindCone || b.kind == surfacegeom.KindCone:
		return "an edge on a retained cone, which no class reads (retained-edge-blend SR1)"
	case a.kind == surfacegeom.KindTorus || b.kind == surfacegeom.KindTorus:
		return "an edge on a retained torus, which no class reads (retained-edge-blend SR1)"
	}
	if a.kind == surfacegeom.KindCylinder && b.kind == surfacegeom.KindCylinder {
		return "an edge between two retained cylinders with parallel axes"
	}
	return fmt.Sprintf("an edge between a retained %s and a retained %s", kindName(a.kind), kindName(b.kind))
}

func kindName(k surfacegeom.SurfaceKind) string {
	if n, ok := kindNames[k]; ok {
		return n
	}
	return fmt.Sprintf("surface of kind %d", int(k))
}
