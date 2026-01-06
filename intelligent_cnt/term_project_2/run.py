from robot import Robot
import argparse
def main(args):
    # this works with threshold = 90, 5 times in a row with clock-wise
    # this works well with CCW
    # model_name = "ccw_cor_cnt_aug_0.89.tflite"

    # finetuned with corner datas
    # CW very well
    model_name = "best_model_finetune_2.tflite"

    # sucks
    # model_name = "best_model_finetune_4.tflite"

    # this fails at the corner w/ CW. maybe retrained. threshold = 95
    # this is good with CCW.
    # model_name = "ccw_cor_cnt_0.93.tflite"

    # model_name = "cw_cnt_aug_0.90.tflite"

    robot = Robot(model_name=model_name, args=args)
    robot.run()
    robot.cleanup()

if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument("--direction", default="cw")
    args = args.parse_args()
    main(args)