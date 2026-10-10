package loftmesh

import "github.com/lestrrat-3d/decad/internal/proofbound"

// MeshProof is the proof record a mesh restating the assembled loft publishes.
// It is composed once from the same triangle set and mass sums as the body.
type MeshProof struct {
	// FacetDeparture bounds the distance from a held facet to its true surface.
	FacetDeparture float64
	// AreaSlack bounds the difference between held and true surface areas.
	AreaSlack float64
	// VolSymDiff bounds the occupied volume difference between mesh and body.
	VolSymDiff float64
}

// IsChorded reports whether any cell's held triangles differ from its denoted
// boundary. A degree-one free-form cell can be chorded at zero departure.
func IsChorded(pairs []LoopPair, sectionDelta, sectionMatchedDelta float64) bool {
	return sectionDelta > 0 || sectionMatchedDelta > 0 || HasUnfacetedCell(pairs)
}

// MeshProofOf composes the payload's facet, area and occupied-volume bounds
// from the assembly and its mass accumulator (tessellation-design §2,
// loft-design §5.2 and §8). These values describe the same held triangle set
// that the payload later restates as a mesh.
//
// FacetDeparture reads the parameter-matched chord departure plus station
// displacement unconditionally. A LineSeg-only build has no chorded mass
// correction but its held facets can still sit a.Delta from the boundary.
// AreaSlack includes the held-to-bilinear leg because the mesh keeps held
// triangles while Area.Value includes the bilinear correction. VolSymDiff
// includes the twist sweep for the same reason, then adds the vertex sweep,
// per-cell wall and skirt legs. Each sum rounds outward at every step.
func MeshProofOf(a Assembly, m *MassAccumulator, sectionMatchedDelta float64) MeshProof {
	return MeshProof{
		FacetDeparture: proofbound.AbsSumUpper(
			ChordCellDeltaUpper(sectionMatchedDelta, a.Delta),
			m.Chorded.MaxTwistOffsetUpper,
		),
		AreaSlack: proofbound.AbsSumUpper(
			m.PerturbAreaSum,
			m.Chorded.TwistAreaAllow,
			m.Chorded.AreaExcess,
			m.Chorded.CapAreaExcess,
		),
		VolSymDiff: proofbound.AbsSumUpper(
			m.SweptVolumeAllow(a.Verts, a.Tris),
			m.Chorded.WallLeg,
			m.Chorded.TwistVolumeUpper,
			m.Chorded.SkirtLeg,
		),
	}
}
