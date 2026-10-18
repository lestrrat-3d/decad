package decad

import (
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/r3"
)

// certifyRevolveLoftSector tries each authenticated finite conical face of
// the Revolve section. The source must certify exactly one face as the
// contact surface; a held geometric resemblance never selects one.
func certifyRevolveLoftSector(blank, sector *Body) (*revolveLoftSectorCertificate, bool) {
	if sector == nil {
		return nil, false
	}
	fp, ok := sector.n.Payload.(facetedPayload)
	if !ok || fp.Xform != r3.Identity() || fp.PointCone == nil ||
		fp.PointCone.Lower == nil || fp.PointCone.Upper == nil ||
		fp.PointCone.Base.PointSection == nil ||
		fp.PointCone.Base.PointSection.Apex != (r3.Vec{}) ||
		fp.PointCone.Base.Xform != r3.Identity() {
		return nil, false
	}
	trim := fp.PointCone
	if !facetedrecord.PointSourceCurrent(trim.Base.PointSection) {
		return nil, false
	}
	var selected *revolveLoftSectorCertificate
	for _, section := range revolveRootCandidates(blank) {
		boundary, ok := revolveproof.CertifyRevolveConnectorBoundary(trim.Base.PointSection,
			section, trim.Base)
		if !ok {
			boundary, ok = revolveproof.CertifyRevolveFitBoundary(trim.Base.PointSection,
				section, trim.Base)
			if !ok {
				continue
			}
		}
		if selected != nil {
			return nil, false
		}
		selected = &revolveLoftSectorCertificate{Blank: bodyNode(blank), Section: section,
			Trimmed: fp, Trim: trim, Base: trim.Base, Boundary: boundary}
	}
	return selected, selected != nil
}
