% verify the environment
function c = isAir(env)
    switch env
        case 'air'
            c = true;
        case 'water'
            c = false;
        otherwise
            c = [];
            error('Environment unknown')
    end
end