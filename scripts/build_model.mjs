#!/usr/bin/env zx

if (argv._.length == 0) {
    console.log(chalk.red("Provide model name"));
    process.exit(1);
}

argv._.forEach((model_name) => {
    console.log(`Processing ${chalk.blue(model_name)}`);
    let model_dir = `${process.env.MODELS_DIR}/${model_name}`;
    $`xacro ${model_dir}/${model_name}.xacro > ${model_dir}/${model_name}.sdf`;
});
