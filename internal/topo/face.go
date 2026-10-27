package topo

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/r3"
)

// CapRecord is the record a planar cap face denotes: the plane-local level Z
// carried through Frame and Xform, every held value read as an exact rational.
// The face lies on {Xform(O + u*U + v*V + Z*N) : u, v real}, displaced from it
// along its normal by at most the face's AxialDelta. It is immutable once a
// producer sets it.
//
// ZDenoted is the exact level Z denotes, where the producer records one
// (agent-docs/stacked-prism-design.md §2.1): the face then lies on the same
// plane taken at ZDenoted. nil records nothing, and a reader falls back to Z
// and AxialDelta.
type CapRecord struct {
	Frame    r3.Frame
	Xform    r3.Transform
	Z        float64
	ZDenoted *big.Rat
}

// Under is the plane record of a copy placed by xform: r itself under the
// identity placement, and nil under any other, since the copy then denotes
// the exact image of r's plane under xform and a held composition of the two
// transforms is not that image. A nil r answers nil.
func (r *CapRecord) Under(xform r3.Transform) *CapRecord {
	if r == nil || xform != r3.Identity() {
		return nil
	}
	return r
}

// Face is the data of the root Face handle, which holds one by
// value.
type Face struct {
	// Area is the held surface area in millimetres².
	Area float64
	// AreaBound is the proven error bound on Area: analytic float evaluation
	// or the composed chord bound for a boolean-built Faceted face.
	AreaBound float64
	// HasCentroid reports whether the face has its own proved area-centroid certificate.
	HasCentroid bool
	// AxialDelta is the proven displacement of this planar face along its own
	// normal. A body-relative ToFace stop carries this selected face's bound
	// into the level it computes; HasAxialDelta distinguishes an exact zero
	// from a face whose payload does not state an axial bound.
	AxialDelta float64
	// HasAxialDelta reports whether the face's payload states an axial bound.
	HasAxialDelta bool
	// HeldPlanar records that a Faceted face descends only from PLANAR source
	// faces, so the face it stands for in the true result is flat — and a rim
	// between two such faces is a straight line, whose chord length is honest.
	// The surface TAG cannot say this for an ordinary mesh boolean: its faces
	// all read Faceted, including the ones that are plainly flat. A structural
	// boolean may instead retain a certified source Plane.
	HeldPlanar bool
	// Reversed is true when the OUTWARD (material-leaving) normal is the
	// surface's geometric normal negated — a hole's cylinder wall, whose
	// material lies outside the cylinder.
	Reversed bool
	// NormalBound is the proven DIMENSIONLESS bound on NormalAt's own answer:
	// how far the surface this face really carries can tilt away from the
	// tagged variant the normal is computed from. It is zero for every face
	// whose own geometry IS its tag, and for a revolve wall, whose departure
	// rides in Denoted instead. It is nonzero for a revolve's cap (its
	// angular displacement, which NormalAt reads only where the cap carries
	// no denoted plane) and for a cap-loop chamfer's band patch: the
	// patch is RULED between two built directrices, and the `Cone` or `Plane`
	// it publishes is that ruled surface only to within a bound measured from
	// the numbers the body publishes for it
	// (root capblend_departure.go, agent-docs/modify-reach-design.md §8.3). The bound is a
	// world-space one, so it covers both a mitered corner's own angular skew and
	// the placement's independent rounding of every coordinate the build emits —
	// which leaves the tag even on a flat patch, and on a circular one whose two
	// windows coincide exactly. A zero term there would omit a direction
	// difference the built surface has. A band patch's bound also carries the
	// turn between that ruled surface and the wall its records denote, since
	// the cap contour it rules to is a solved offset held within a proven
	// displacement (capband.DenotedNormalAllow). NormalAt separately composes
	// its arithmetic proof (root normal_bound.go).
	NormalBound float64
	// RetainedTag marks a face of a mesh Boolean result whose Surface is the
	// recorded surface of the operand face it descends from, while its loops
	// and area stay the held polygons (agent-docs/faceted-source-retention-design.md
	// §2). The tag is a bounded stand-in: NormalBound covers both the tag's
	// own departure (TagDeparture) and the held displacement of this face.
	RetainedTag bool
	// HeldNormalBound is the normal bound a retained denoted face adds, on top
	// of its denoted proof, for facets that sit up to the face's displacement
	// from the true patch. A retained face without a denoted patch folds the
	// same term into NormalBound instead. Zero for every other face.
	HeldNormalBound float64
	// TagDeparture is the part of NormalBound that states how far the
	// retained tag itself tilts from the true surface, as a placement or
	// similarity leaves it; zero for a tag that is its operand's own
	// geometry. A later Boolean reads it back so the held-displacement term is
	// recomputed against the new face, not accumulated.
	TagDeparture float64
	// Denoted is the surface a revolve wall or cap DENOTES — a wall's
	// recorded meridian segment swept about its recorded axis, or a cap's
	// plane through that axis at its end's recorded angle, enclosed exactly
	// (surfacenormal.Revolved) — where the tag is only a float re-expression
	// of it. NormalAt judges such a face's normal against it rather than
	// against the tag, so the bound covers the tag's whole departure; nil
	// for every face whose tag is its own denotation. It is not
	// NormalBound: no dimensionless term states the departure for every p.
	// Stitch's flux arms integrate the tag, so they admit a revolve wall only
	// where its tag is exactly this surface (internal/stitchrecord/flux.go's
	// fluxTagsDenoted).
	Denoted *surfacenormal.Revolved
	// Surface is the face's tagged geometry in its internal variant. Root
	// Face.Surface converts it to the public variant with publicSurface.
	Surface surfacegeom.Surface
	// CapRecord is the plane this planar face denotes, nil where the producer
	// states none. A ToFace stop reads it; a face without it is not a stop.
	CapRecord *CapRecord

	// Origins is every feature role that created the face, in the order they
	// were merged. A merged face carries all contributing roles. Root
	// Face.Origins converts it to the public FeatureRef form.
	Origins []provenance.Ref
	// Source is the record of the faceted group this face was assembled from,
	// nil for any other face. It only traces origin and admits nothing.
	Source *provenance.SourcePatch
	// Centroid is present only for a face with its own proved area-centroid
	// certificate; whole-group readings are not copied onto split patches.
	// Root Face.Centroid converts it to the public VecMeasurement form.
	Centroid measurement.VecMeasurement
	// Loops are the nodes of the face's boundary loops, the outer loop
	// first. Root Face.Loops maps them back to their handles.
	Loops []*Loop
	// Body is the node of the body that holds this face, nil for a face no
	// body holds. Root maps it back to its handle through Body.Owner.
	Body *Body
	// Owner is the root Face handle that holds this node by value. Root
	// sets it when the handle is born and maps a node back to its handle
	// through it; nothing else writes it.
	Owner any
}

// DenotedUnder carries a face's denoted surface onto a copy placed by xform:
// the exact image of the source face's own, never a re-reading of the copy's
// rounded tag.
func (f *Face) DenotedUnder(xform r3.Transform) *surfacenormal.Revolved {
	if f.Denoted == nil || xform == r3.Identity() {
		return f.Denoted
	}
	out := f.Denoted.Transformed(xform)
	return &out
}

// CoEdgeDirection returns f's own forward flag for its use of e, and whether f
// actually uses e at all.
func (f *Face) CoEdgeDirection(e *Edge) (bool, bool) {
	for _, l := range f.Loops {
		for _, ce := range l.CoEdges {
			if ce.Edge == e {
				return ce.Forward, true
			}
		}
	}
	return false, false
}
