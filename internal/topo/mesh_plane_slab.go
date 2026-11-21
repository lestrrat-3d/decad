package topo

import "github.com/lestrrat-3d/r3"

// MeshPlaneSlab bounds the normal distance of an exact mesh face and its
// Boolean fragments to Frame. It does not identify an analytic surface.
type MeshPlaneSlab struct {
	Frame    r3.Frame
	Distance float64
}

// SetMeshPlaneSlab binds a finite positional proof to this particular face.
// A builder copying a Face by value cannot accidentally claim its source plane.
func (f *Face) SetMeshPlaneSlab(slab *MeshPlaneSlab) {
	f.meshSlab = slab
	if slab != nil {
		f.meshSlabOwner = f
	} else {
		f.meshSlabOwner = nil
	}
}

// MeshPlaneSlab returns only a proof bound to this face at construction.
func (f *Face) MeshPlaneSlab() *MeshPlaneSlab {
	if f == nil || f.meshSlabOwner != f {
		return nil
	}
	return f.meshSlab
}
