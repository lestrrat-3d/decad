package sweeprecord

import (
	"strings"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

func FinishStraight(body *topo.Body, payload Payload) {
	if built, ok := body.Payload.(prismrecord.Payload); ok {
		payload.Prism = built
	}
	prefixSpanZeroRole(body)
	body.Payload = payload
}

func FinishChain(body *topo.Body, payload ChainPayload) {
	if built, ok := body.Payload.(chainrecord.Payload); ok {
		payload.Chain = built
	}
	if built, ok := body.Payload.(revolveplan.ChainPayload); ok {
		payload.Revolve = built
	}
	prefixSpanZeroRole(body)
	body.Payload = payload
}

// prefixSpanZeroRole rewrites every wall role a one-span reduction minted
// so it carries Table B's own path-span index ahead of the section's loop and
// segment indices: side(i,j) becomes side(0,i,j). The reduction builds through
// the prism or chain-prism evaluator, which knows nothing of a path span, so
// the index is restored here rather than threaded through that builder.
func prefixSpanZeroRole(body *topo.Body) {
	for _, face := range topobuild.Faces(body) {
		for i, origin := range face.Origins {
			if suffix, ok := strings.CutPrefix(origin.Role, "side("); ok {
				origin.Role = "side(0," + suffix
				face.Origins[i] = origin
			}
		}
	}
}

// FinishArc restores Table B's own role vocabulary on an arc
// reduction, which builds through the Revolve evaluator and so mints roles
// that know nothing of a path. Every wall role gains the path-span index ahead
// of the section's loop and segment indices, exactly as
// prefixSpanZeroRole does for a straight reduction. A closed full-circle
// reduction adds the second span's role to each seam-free full-turn wall. A
// span whose Revolve axis was reoriented (ReverseArcCaps) has its cap roles swapped back
// into PATH order, so capStart always names the section the path leaves and
// capEnd the one it arrives at — the orientation compositesweep.Assemble's own
// join pairing reads when it takes one span's end cap against the next span's
// start cap. A closed full-circle reduction has no cap.
func FinishArc(body *topo.Body, payload Payload) {
	if built, ok := body.Payload.(revolveplan.Payload); ok {
		payload.Revolve = built
	}
	for _, face := range topobuild.Faces(body) {
		origins := make([]provenance.Ref, 0, len(face.Origins)*2)
		for _, origin := range face.Origins {
			switch {
			case strings.HasPrefix(origin.Role, "side("):
				suffix := strings.TrimPrefix(origin.Role, "side(")
				origin.Role = "side(0," + suffix
				if payload.ClosedCircle {
					second := origin
					second.Role = "side(1," + suffix
					origins = append(origins, origin, second)
					continue
				}
			case payload.ReverseArcCaps && origin.Role == provenance.RoleCapStart:
				origin.Role = provenance.RoleCapEnd
			case payload.ReverseArcCaps && origin.Role == provenance.RoleCapEnd:
				origin.Role = provenance.RoleCapStart
			}
			origins = append(origins, origin)
		}
		face.Origins = origins
	}
	body.Payload = payload
}

func RestoreArcCapRoles(body *topo.Body, reverse bool) {
	if !reverse {
		return
	}
	for _, face := range topobuild.Faces(body) {
		for i, origin := range face.Origins {
			origin.Role = compositesweep.SwapCapRole(origin.Role, provenance.RoleCapStart, provenance.RoleCapEnd)
			face.Origins[i] = origin
		}
	}
}
