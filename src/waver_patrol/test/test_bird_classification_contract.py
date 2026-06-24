from waver_patrol.perception.bird_classification import (
    classify_candidate,
    detector_model_state,
    normalize_class_name,
    parse_class_aliases,
)


def test_bird_class_normalization_accepts_case_variants():
    for name in ("bird", "Bird", "BIRD"):
        decision = classify_candidate(name, 0.81, bird_threshold=0.75)
        assert decision.normalized_class == "bird"
        assert decision.bird_confirmed
        assert not decision.non_bird


def test_known_non_bird_classes_are_rejected_for_sound_or_goal():
    for name in ("person", "car", "truck", "drone", "robot"):
        decision = classify_candidate(name, 0.95)
        assert not decision.bird_confirmed
        assert decision.non_bird


def test_unknown_and_low_confidence_never_confirm_bird():
    assert not classify_candidate("bird", 0.2, bird_threshold=0.75).bird_confirmed
    assert classify_candidate("bird", 0.2, bird_threshold=0.75).unknown
    assert not classify_candidate("sparrow_like_noise", 0.99).bird_confirmed


def test_alias_table_is_explicit_and_conservative():
    aliases = parse_class_aliases(["sparrow:bird", "plane:irrelevant"])
    assert normalize_class_name("SPARROW", aliases) == "bird"
    assert normalize_class_name("plane", aliases) == "irrelevant"
    assert classify_candidate("plane", 0.99, aliases=aliases).non_bird


def test_missing_real_model_path_is_not_success(tmp_path):
    assert detector_model_state("", required=True) == "MODEL_MISSING"
    missing = tmp_path / "missing.pt"
    assert detector_model_state(str(missing), required=True).startswith("MODEL_NOT_FOUND")
    model = tmp_path / "bird.pt"
    model.write_bytes(b"placeholder")
    assert detector_model_state(str(model), required=True) == "MODEL_READY"
