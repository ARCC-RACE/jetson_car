from tensorflow.python.compiler.tensorrt import trt_convert as trt
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--data", help="directory to the dataset for Int8 calibration", type=string)
parser.add_argument("-m", "--model", help="directory to the tf saved model that you would like to convert to RT", type=string)
args = parser.parse_args()

sys.path.append(args.model)
import utils


print(utils.INPUT_SHAPE)

conversion_params = trt.DEFAULT_TRT_CONVERSION_PARAMS
conversion_params = conversion_params._replace(max_workspace_size_bytes=(1 << 32))
conversion_params = conversion_params._replace(precision_mode="INT8")
conversion_params = conversion_params._replace(maximum_cached_engines=100)
conversion_params = conversion_params._replace(use_calibration=True)


def my_calibration_input_fn():
    for i in range(20):
        image, _ = utils.preprocess_data(utils.load_image(args.data, os.path.join("color_images", x[i])))
        yield image,


converter = trt.TrtGraphConverterV2(input_saved_model_dir=args.model, conversion_params=conversion_params)

gen = my_calibration_input_fn()

converter.convert(calibration_input_fn=my_calibration_input_fn)
converter.build(my_calibration_input_fn)

if not os.path.isdir(os.path.join(args.model, "rt")):
    os.mkdir(os.path.join(args.model, "rt"))

converter.save(os.path.join(args.model, "rt"))