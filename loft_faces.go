package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/r3"
)

const maxLoftFacesSections = 32

// LoftFaces builds one solid through ordered complete caps of the same
// authenticated point-loft Split source (docs/loft-face-sections-design.md).
func LoftFaces(ctx context.Context, d *Document, sections ...*Face) (*Body, error) {
	if ctx == nil || d == nil || len(sections) < 2 || len(sections) > maxLoftFacesSections {
		return nil, fmt.Errorf("%w: LoftFaces needs a context, document and 2 to %d cap faces",
			ErrDegenerate, maxLoftFacesSections)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	selected, source, err := collectLoftFaces(d, sections)
	if err != nil {
		return nil, err
	}
	return buildLoftFaces(ctx, d, selected, source, nil)
}

// buildLoftFaces has revolveloftfaces.BuildLoftFaces loft through the
// collected sections, then commits the body.
func buildLoftFaces(ctx context.Context, d *Document, sections []revolveloftfaces.LoftFacesSection,
	source *pointSectionRecord, tangent *loftface.TangentEnd) (*Body, error) {
	ref := d.nextProducerID()
	body, err := bodyResult(revolveloftfaces.BuildLoftFaces(ctx, envOf(documentNode(d), ref), sections, source, tangent))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// collectLoftFaces checks every selected face against the document and its
// Split slab, which reads root Face and Body handles, and returns the
// sections revolveloftfaces.BuildLoftFaces lofts through.
func collectLoftFaces(d *Document, faces []*Face) ([]revolveloftfaces.LoftFacesSection, *pointSectionRecord, error) {
	sections := make([]revolveloftfaces.LoftFacesSection, len(faces))
	var source *pointSectionRecord
	seen := make(map[*Body]bool, len(faces)-1)
	for i, face := range faces {
		if face == nil || face.n.Body == nil {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d is not a body face", ErrDegenerate, i)
		}
		body := bodyHandle(face.n.Body)
		if err := d.requireLive(body); err != nil {
			return nil, nil, err
		}
		fp, ok := body.n.Payload.(facetedPayload)
		if !ok || fp.PointPlane == nil || fp.PointPlane.Base.PointSection == nil ||
			len(fp.Verts) == 0 || len(fp.FaceOf) != len(fp.Tris) {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d has no authenticated Split cap", ErrUnsupported, i)
		}
		record := fp.PointPlane.Base.PointSection
		if !facetedrecord.PointSourceCurrent(record) {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d has a stale source profile", ErrStaleProfile, i)
		}
		if source == nil {
			source = record
		} else if source != record {
			return nil, nil, fmt.Errorf("%w: LoftFaces sections have different point-loft sources", ErrUnsupported)
		}
		if fp.PointSimilarity != nil && fp.PointSimilarity.Slice != fp.PointPlane {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d has broken similarity provenance", ErrUnsupported, i)
		}
		faceIndex := -1
		for j, candidate := range body.Faces() {
			if candidate == face {
				faceIndex = j
				break
			}
		}
		if faceIndex < 0 {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d is not in its body", ErrUnsupported, i)
		}
		group := -1
		for j, owner := range fp.FaceOf {
			if owner == faceIndex {
				group = fp.Src[j]
				break
			}
		}
		want := len(fp.Groups) - 1
		if i == 0 {
			want--
		}
		if group != want || (i == 0 && fp.PointPlane.Lower == nil) {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d is not the required Split cap", ErrUnsupported, i)
		}
		if complete, ok := pointCapGroupFace(body, fp, want); !ok || complete != face {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d is not one complete cap patch", ErrUnsupported, i)
		}
		if i == 1 && body != bodyHandle(faces[0].n.Body) {
			return nil, nil, fmt.Errorf("%w: LoftFaces first toe and heel must share one slab", ErrUnsupported)
		}
		if i >= 1 {
			if seen[body] {
				return nil, nil, fmt.Errorf("%w: LoftFaces repeats a heel slab", ErrUnsupported)
			}
			seen[body] = true
		}
		n := len(fp.PointPlane.Base.Verts) - 1
		if n < 3 || len(fp.Verts) != 2*n || len(fp.PointPlane.Base.Tris) <= n {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d has no complete station ring", ErrUnsupported, i)
		}
		ring := fp.Verts[n:]
		limit := fp.PointPlane.Upper
		if i == 0 {
			ring = fp.Verts[:n]
			limit = fp.PointPlane.Lower
		}
		sections[i] = revolveloftfaces.LoftFacesSection{Mesh: fp, Slice: fp.PointPlane, Limit: limit,
			Group: group, Ring: ring, Surface: face.n.Surface}
	}
	return sections, source, nil
}

// The LoftFaces cap-map carriers live in internal/loftface.
type loftFaceRatVec = loftface.RatVec

type loftFaceMap = loftface.Map

type loftFaceInterval = loftface.Interval

func lfZero() *big.Rat { return new(big.Rat) }

func lfOne() *big.Rat { return big.NewRat(1, 1) }

func lfF(v float64) *big.Rat { return proofarith.FloatRat(v) }

// lfAdd, lfSub, lfMul and lfDiv return a fresh Rat and read their operands only.
// Operands must be in lowest terms: the result always equals the exact value,
// but is in lowest terms only for reduced operands (a 3/9 operand gives 3/9
// where big.Rat gives 1/3). Every producer in this package reduces: FloatRat
// through lfF, big.NewRat literals, lfOne, lfZero, and
// results of these helpers and of raw Quo, Abs and Neg. The guard
// TestNoRatWritesThroughNumDenom (internal/proof) bars in-place Num and Denom
// writes outside internal/proof. lfDiv by zero panics as Rat.Quo does.
func lfAdd(a, b *big.Rat) *big.Rat { return proofarith.AddRat(new(big.Rat), a, b) }

func lfSub(a, b *big.Rat) *big.Rat { return proofarith.SubRat(new(big.Rat), a, b) }

func lfMul(a, b *big.Rat) *big.Rat { return proofarith.MulRat(new(big.Rat), a, b) }

func lfDiv(a, b *big.Rat) *big.Rat { return proofarith.QuoRat(new(big.Rat), a, b) }

func lfAbs(a *big.Rat) *big.Rat { return new(big.Rat).Abs(a) }

// ConeTangentEnd names the cone that the last, constructed LoftFacesToConeTangent
// section touches along one ruling.
type ConeTangentEnd struct {
	// Cone is a live, unplaced, full straight-meridian Revolve cone about world X.
	Cone *Body
	// Radial is the direction of the touching ruling's radius. Its X must be 0
	// and its (Y, Z) finite and nonzero.
	Radial r3.Vec
}

// LoftFacesToConeTangent lofts through 1 to 31 selected caps, then a
// constructed last section tangent to end.Cone along one ruling
// (docs/loft-face-sections-design.md, "Cone-tangent end section").
//
// sections follow LoftFaces: the least slab's toe cap, then heel caps in
// source order. The last section is built from the cone, not selected. Its
// plane is exactly tangent to the cone's exact apex and slope, with the
// source origin at the cone apex, so it fits a point-loft source whose plane
// is that tangent plane. Any other source still goes through the ordinary
// embedding gates, which refuse an inconsistent end. The end face carries the
// cap-end role of the new loft and a rounded Plane tag. Cut and Intersect
// continue from the result as they do from LoftFaces.
//
// The cone must be live in the sections' document. A nil context, cone or
// section list of the wrong length, a Radial with a nonzero X or without a
// finite nonzero (Y, Z), or a cone that is not an exact point cone is
// ErrDegenerate or ErrUnsupported. Selection failures are those of
// LoftFaces. A canceled ctx returns ctx.Err() before the document changes.
func LoftFacesToConeTangent(ctx context.Context, end ConeTangentEnd, sections ...*Face) (*Body, error) {
	if ctx == nil || end.Cone == nil || len(sections) < 1 || len(sections) > maxLoftFacesSections-1 {
		return nil, fmt.Errorf("%w: LoftFacesToConeTangent needs a context, a cone and 1 to %d cap faces",
			ErrDegenerate, maxLoftFacesSections-1)
	}
	d := documentHandle(end.Cone.n.Doc)
	if d == nil {
		return nil, fmt.Errorf("%w: the cone belongs to no document", ErrDegenerate)
	}
	if err := d.requireLive(end.Cone); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	cone, ok := revolveplan.PointConeSource(&end.Cone.n)
	if !ok {
		return nil, fmt.Errorf("%w: LoftFacesToConeTangent needs an unplaced full straight-meridian cone about X",
			ErrUnsupported)
	}
	if !proofbound.FiniteVec(end.Radial) || end.Radial.X != 0 || (end.Radial.Y == 0 && end.Radial.Z == 0) {
		return nil, fmt.Errorf("%w: LoftFacesToConeTangent needs a finite radial direction with X = 0 and (Y, Z) nonzero",
			ErrDegenerate)
	}
	selected, source, err := collectLoftFaces(d, sections)
	if err != nil {
		return nil, err
	}
	tangent, err := loftface.NewTangentEnd(cone.Apex, cone.Slope, end.Radial.Y, end.Radial.Z)
	if err != nil {
		return nil, fmt.Errorf("%w: %s", ErrUnsupported, err)
	}
	endMap := tangent.Map()
	ring := make([]r3.Vec, len(source.SourceStation))
	for j, station := range source.SourceStation {
		u, v := lfF(station.U), lfF(station.V)
		ring[j] = loftface.RoundPoint(endMap.At(u, v))
	}
	selected = append(selected, revolveloftfaces.LoftFacesSection{Slice: selected[0].Slice, Ring: ring, Constructed: &endMap})
	return buildLoftFaces(ctx, d, selected, source, &tangent)
}
