#!/usr/bin/env zx

if (argv._.length == 0) {
    console.log(chalk.red("Provide model name"));
    process.exit(1);
}

function process_xacro_file(filename) {
    const realname = filename.slice(0, -6);
    if (!realname.includes('.')) {
        return;
    }
    $`xacro ${filename} > ${realname}`;
}


argv._.forEach(async (model_name) => {
    console.log(`Processing ${chalk.blue(model_name)}`);
    const model_dir = `${process.env.MODELS_DIR}/${model_name}`;

    const targets = (await $`find ${model_dir} -name "*.xacro"`.quiet()).stdout.slice(0, -1).split('\n');
    targets.forEach(process_xacro_file);
});
