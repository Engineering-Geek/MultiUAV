from environments.mujoco_xmls import make_model, ObjectXML
from mujoco.viewer import launch


def test_make_model():
    model = make_model(
        n_drones=2,
        objects=[]
    )
    launch(model)


if __name__ == "__main__":
    test_make_model()
