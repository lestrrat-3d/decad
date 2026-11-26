package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/modifyoption"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// The shared settings of the modify commands. Each command declares its own
// one-line method over one of these setters (agent-docs/command-api-design.md
// §6); a method stores its value, and the operation's gate refuses a fault of
// it at today's position (§4, rules A6 and A9).

// tangentChainSetting is the TangentChain setting of FilletCommand and
// ChamferCommand. Calling the method twice is the same as calling it once.
type tangentChainSetting struct{ on bool }

func (s *tangentChainSetting) set() { s.on = true }

type asymmetricChamferOpts = modifyoption.Asymmetric[*FaceQuery]

// asymmetricChamferSetting holds the Asymmetric setting of a chamfer: the
// copied reference with the second distance, and the error validating the
// distance. The latest call replaces both.
type asymmetricChamferSetting struct {
	slot opcommand.Slot[asymmetricChamferOpts]
}

func (s *asymmetricChamferSetting) set(reference FaceSelector, otherDistance units.Value) {
	a, err := newAsymmetricChamfer(reference, otherDistance)
	s.slot.Set(a, err)
}

// newAsymmetricChamfer copies reference, so a later change to the caller's
// query does not reach the chamfer, and validates the second distance.
func newAsymmetricChamfer(reference FaceSelector, otherDistance units.Value) (asymmetricChamferOpts, error) {
	a := asymmetricChamferOpts{Other: otherDistance}
	switch q := reference.(type) {
	case nil:
	case *FaceQuery:
		a.Reference = copyFaceQuery(q)
	default:
		a.Foreign = fmt.Sprintf(`%T`, reference)
	}
	return modifyoption.ValidateAsymmetric(a)
}

// resolveAsymmetricReference is stage 3 of the reach gate order: it resolves
// the asymmetric chamfer's reference against the receiver and pairs every
// expanded edge with its one reference face (agent-docs/modify-design.md Part R §6,
// SX3). A nil reference is errNilSelector and a foreign one ErrDegenerate,
// the selector errors the seed query takes. An edge with no adjacent face in
// the resolved set, or with both, and a resolved face adjacent to no
// expanded edge, are ErrCardinality.
func resolveAsymmetricReference(b *Body, a *asymmetricChamferOpts, edges []*Edge) (map[*topo.Edge]*topo.Face, error) {
	if a.Foreign != "" {
		return nil, fmt.Errorf(`%w: the asymmetric chamfer's reference is not a decad face query (%s)`, ErrDegenerate, a.Foreign)
	}
	if a.Reference == nil {
		return nil, fmt.Errorf(`%w (the asymmetric chamfer's reference)`, errNilSelector)
	}
	faces, err := a.Reference.SelectFaces(b)
	if err != nil {
		return nil, err
	}
	resolved := make(map[*Face]struct{}, len(faces))
	for _, f := range faces {
		resolved[f] = struct{}{}
	}
	touched := make(map[*Face]struct{}, len(faces))
	refs := make(map[*topo.Edge]*topo.Face, len(edges))
	for ei, e := range edges {
		var ref *Face
		count := 0
		for _, f := range e.n.Faces {
			if _, ok := resolved[faceHandle(f)]; ok {
				ref = faceHandle(f)
				count++
			}
		}
		if count != 1 {
			return nil, fmt.Errorf(`%w: the asymmetric chamfer's reference %s names %d of the faces adjacent to %s; it must name exactly one (modify-reach SX3)`,
				ErrCardinality, a.Reference, count, modifyselect.EdgeContext(ei, edgeNode(e)))
		}
		refs[edgeNode(e)] = faceNode(ref)
		touched[ref] = struct{}{}
	}
	if len(touched) != len(resolved) {
		return nil, fmt.Errorf(`%w: the asymmetric chamfer's reference %s matched %d faces and only %d of them is adjacent to a chamfered edge (modify-reach SX3)`,
			ErrCardinality, a.Reference, len(resolved), len(touched))
	}
	return refs, nil
}

// errNilBody is the refusal of a nil body, which every modify command's
// constructor records because the operation reports it right after the nil
// ctx check.
var errNilBody = fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
