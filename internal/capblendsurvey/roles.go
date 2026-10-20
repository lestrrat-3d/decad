package capblendsurvey

import (
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// listVerdict keeps a proven opposing face beside any undecided face.
func listVerdict(faces *[]*topo.Face, undecided *bool, f *topo.Face, verdict survey2d.PullVerdict, ok bool) bool {
	if !ok {
		return false
	}
	switch verdict {
	case survey2d.PullOpposes:
		*faces = append(*faces, f)
	case survey2d.PullUndecided:
		*undecided = true
	}
	return true
}
