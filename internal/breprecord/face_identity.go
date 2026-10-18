package breprecord

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/topo"
)

// MatchRecordFace ties a live reference face to its receiver producer and record role.
func MatchRecordFace(receiver *topo.Body, ref *topo.Face, roles []string, index int) bool {
	if ref == nil {
		return false
	}
	faceRole := fmt.Sprintf("face(%d)", index)
	wallRole := fmt.Sprintf("wall(%d)", index)
	if roles != nil {
		if index < 0 || index >= len(roles) {
			return false
		}
		faceRole, wallRole = roles[index], roles[index]
	}
	for _, origin := range ref.Origins {
		if origin.Producer == receiver.Origin.Producer && (origin.Role == faceRole || origin.Role == wallRole) {
			return true
		}
	}
	return false
}
