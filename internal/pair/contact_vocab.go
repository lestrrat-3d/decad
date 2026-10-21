package pair

import "github.com/lestrrat-3d/decad/report"

// ContactReasonOf names a pair kernel's missing certificate as a contact
// reason; NoReason, and any reason it does not name, is ContactNoReason.
func ContactReasonOf(reason Reason) report.ContactReason {
	switch reason {
	case NoGapProof:
		return report.ContactNoGapProof
	case AmbiguousFeature:
		return report.ContactAmbiguousFeature
	case PointTooCoarse:
		return report.ContactPointTooCoarse
	case PayloadUnsupported:
		return report.ContactPayloadUnsupported
	case NoNormalProof:
		return report.ContactNoNormalProof
	default:
		return report.ContactNoReason
	}
}

// RelationOfContact is the relation the exact pair kernels read r as: a band
// or an undecided relation reads as undecided.
func RelationOfContact(r report.ContactRelation) Relation {
	switch r {
	case report.ContactSeparated:
		return Separated
	case report.ContactTouching:
		return Touching
	case report.ContactOverlapping:
		return Overlapping
	default:
		return Undecided
	}
}
