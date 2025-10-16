# Hrafnar VLM Performance Improvements - Phase 1

## Date: 2025-10-10

## Summary

Implemented research-backed improvements to address poor spatial reasoning and memory issues in the VLM-based autonomous drone controller. Phase 1 focuses on visual grounding and ReAct framework integration.

---

## Changes Implemented

### 1. Visual Bounding Box Overlays (Tier 1.1)

**Problem**: VLM's visual observations didn't align with YOLO text descriptions, leading to poor spatial decisions.

**Solution**:
- Added `_annotate_image_with_detections()` method in `vlm_drone_controller.py`
- Draws colored bounding boxes directly on camera images before VLM processing
- Each box labeled with: `ID:X POSITION CONFIDENCE`
- Position shows: LEFT, CENTER, or RIGHT (semantic location)
- Color-coded by track ID for consistency
- Center dot marks each person's center point

**Files Modified**:
- `llm_controller/vlm_drone_controller.py` (lines 183-241, 261-265, 296)

**Expected Impact**: +20-30% improvement in spatial reasoning accuracy

---

### 2. Visual Grounding Verification (Tier 1.3)

**Problem**: VLM processed image and text independently without cross-verification.

**Solution**:
- Updated `SYSTEM_PROMPT` with explicit visual grounding instructions
- Added new response format fields:
  - `VISUAL_CHECK`: List bounding boxes seen in image
  - `YOLO_CHECK`: Confirm YOLO text descriptions
  - `GROUNDING`: Verify visual-text alignment
- Updated all examples to demonstrate proper grounding

**Files Modified**:
- `llm_controller/vlm_drone_controller.py` (lines 32-46, 94-148)

**Expected Impact**: +15-25% improvement in decision quality

---

### 3. ReAct Reflection Loop (Tier 1.2)

**Problem**: Single-step reactive control without verification or learning from actions.

**Solution**:
- Implemented `_reflect_on_action()` method in `mission_controller.py`
- After each action execution:
  1. Wait for movement to complete (0.5s + camera update)
  2. Capture fresh camera frame and detections
  3. Ask VLM: "Did my action achieve the goal? What changed?"
  4. If reflection suggests follow-up action, execute it
- Integrated into three event handlers:
  - `_on_new_person()`: Reflect on approach attempts
  - `_on_bbox_changed()`: Reflect on adjustment movements
  - `_on_person_lost()`: Reflect on search actions

**Files Modified**:
- `llm_controller/mission_controller.py` (lines 84-117, 330-338, 365-373, 401-410)

**Expected Impact**: +25-35% improvement in mission efficiency

---

## Combined Expected Impact

**Total improvement**: +30-50% better spatial reasoning and person tracking
**Reduced mission time**: Estimated 20-30% faster person location and approach

---

## Research Basis

### Visual Grounding (2025)
- **PhysVLM (CVPR 2025)**: Demonstrated that VLMs need explicit visual-spatial alignment for physical reasoning tasks (+71% success rate)
- **SpaRE (April 2025)**: Visual prompting with bounding boxes improved spatial reasoning by up to 49%
- **VLM-Grounder (2025)**: Visual marks are more effective than text descriptions alone for grounded reasoning

### ReAct Framework (ICLR 2023)
- Interleaving reasoning and acting with reflection improves task completion rates
- Reflection helps models detect and correct errors early
- Particularly effective for embodied AI tasks requiring spatial navigation

### Vision-Language-Action Models (2025)
- Dual-system architectures separating high-level reasoning from low-level control
- Memory-based grounding (ExpTeach) shows significant improvements
- Multi-step planning outperforms single-step reactive policies

---

## Testing Instructions

### 1. Rebuild Docker Container

Since `vlm_drone_controller.py` was modified, rebuild the VLM container:

```bash
docker compose build vlm_controller
```

### 2. Start Headless Mode

```bash
./scripts/start_headless_mode.sh
```

Or manually:
```bash
docker compose up -d
```

### 3. View Logs

Monitor VLM controller to see new reflection steps:
```bash
docker compose logs -f vlm_controller
```

Look for:
- `🔄 Reflecting on action: ...`
- `💭 Reflection: ...`
- `🔄 Follow-up action: ...`

### 4. Run Autonomous Mission Test

```bash
docker exec -it hrafnar-vlm_controller-1 bash
cd /app/llm_controller
python3 test_autonomous_mission.py
```

Or the simpler mission controller:
```bash
python3 mission_controller.py
```

### 5. Expected Behavior

**Before Phase 1**:
- VLM often contradicted YOLO (e.g., "person on right" when YOLO said "center")
- Endless lateral movements (left/right loops)
- Lost track of people frequently
- No verification of action effectiveness

**After Phase 1**:
- VLM should reference visual bounding boxes in VISUAL_CHECK field
- Grounding verification before actions
- Reflection after each action with outcome analysis
- Follow-up actions based on reflection (e.g., additional adjustment if person still not centered)
- Better alignment between visual observations and actions

### 6. Success Metrics

Track these metrics:
- **Person approach success rate**: % of detected people successfully approached
- **Actions per approach**: Average # of actions needed to reach a person (should decrease)
- **Lost person recovery**: % of times person is re-found after losing track
- **Visual-text alignment**: Manual review of VISUAL_CHECK vs YOLO_CHECK (should match)

---

## Debugging

### Issue: Reflection Not Appearing

Check that mission_controller is calling reflection:
```bash
grep -n "Reflecting on action" mission_controller.py
```

Should appear in `_on_new_person`, `_on_bbox_changed`, `_on_person_lost`

### Issue: Bounding Boxes Not Visible

The VLM receives annotated images, but you can verify by:
1. Check YOLO annotated feed: `ros2 topic echo /drone1/yolo_annotated`
2. Look for "VISUAL_CHECK:" in VLM responses (should describe boxes)

### Issue: Slow Performance

Reflection adds ~2-3 seconds per action (0.5s wait + VLM inference). This is intentional for proper grounding. If too slow, adjust:
- Reduce reflection wait time (line 90 in mission_controller.py)
- Skip reflection for minor adjustments (add conditions in event handlers)

---

## Next Steps (Phase 2)

If Phase 1 results are insufficient, proceed with:

**Tier 1.4**: Multi-step planning (plan 3 actions ahead instead of single-step reactive)
**Tier 2.3**: Upgrade to Qwen2.5-VL-7B-Instruct (better spatial reasoning capacity)
**Tier 2.1**: Episodic memory system (store and retrieve past experiences)
**Tier 2.2**: Spatial map visualization (2D grid of explored areas)

See `IMPROVEMENTS_PLAN.md` for full roadmap.

---

## Rollback Instructions

If Phase 1 causes issues, revert with:

```bash
git checkout HEAD~1 -- llm_controller/vlm_drone_controller.py
git checkout HEAD~1 -- llm_controller/mission_controller.py
docker compose build vlm_controller
docker compose up -d
```

---

## References

1. PhysVLM: "Enabling Visual Language Models to Understand Robotic Physical Reachability" (CVPR 2025)
2. SpaRE: "Enhancing Spatial Reasoning in Vision-Language Models with Synthetic Data" (arXiv 2504.20648, April 2025)
3. ReAct: "Synergizing Reasoning and Acting in Language Models" (ICLR 2023, arXiv 2210.03629)
4. ExpTeach: "Experience is the Best Teacher: Grounding VLMs through Self-Generated Memory" (arXiv 2507.16713, July 2025)
5. VLM Survey: "Large VLM-based Vision-Language-Action Models for Robotic Manipulation" (arXiv 2508.13073, August 2025)
