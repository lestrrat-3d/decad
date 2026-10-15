package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

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
	cone, ok := exactPointCone(end.Cone)
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
		u, v := lfStationUV(station)
		ring[j] = loftface.RoundPoint(endMap.At(u, v))
	}
	selected = append(selected, loftFaceSection{slice: selected[0].slice, ring: ring, constructed: &endMap})
	return buildLoftFaces(ctx, d, selected, source, &tangent)
}
