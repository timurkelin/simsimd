close all;
clearvars;
rng( 'shuffle', 'twister' );

dm_size = 4096; % total dm size
ram_num = 4;

dm_init_prm = [ ...
    struct( 'var', 'seg_init_data_a', 'len',  200 ); ...
    struct( 'var', 'seg_init_data_b', 'len',  400 ); ...
    struct( 'var', 'seg_init_data_c', 'len',  200 ); ...
    struct( 'var', 'seg_init_data_d', 'len',  400 )];

for init_idx = 1:numel( dm_init_prm )
    
    ram_len = floor( dm_init_prm( init_idx ).len );
    
    ram_data = complex( zeros( ram_len, ram_num ));
    ram_csum = complex( zeros(       1, ram_num )); % Initialize checksum
    
    for ram_addr = 1:( ram_len - 1 )
        for ram_idx = 1:ram_num
            if ram_idx == 1
                ram_data( ram_addr, ram_idx ) = complex( ( ram_addr - 1 ), -( ram_addr - 1 ));
            else
                ram_data( ram_addr, ram_idx ) = double( complex( single( randn( 1 )), single( randn( 1 ))));   
            end
        end
        
        ram_csum = ram_csum + ram_data( ram_addr, : ); % Update checksum
    end
    
    ram_data( ram_len, : ) = ram_csum; % Save checksum
    
    eval( [dm_init_prm( init_idx ).var, ' = reshape( ram_data.'', [], 1 );'] );
    
    if( init_idx == 1 )
        save( 'seg_init.mat', dm_init_prm( init_idx ).var );
    else
        save( 'seg_init.mat', dm_init_prm( init_idx ).var, '-append' );
    end
end
