% Class for the FFT processor 

classdef fft_c < handle
  
   properties ( Constant = true, GetAccess = private )
      class_id = 'fft_c';
   end % properties    
   
   properties ( SetAccess = private, GetAccess = private )
      run_fpath;  % Path
      run_prm;    % parameters
   end % properties
   
   properties ( Constant = true, GetAccess = private )
      dm_ways = 4;   % Number of DM tiles in each of 2 banks
   end % properties    
   
   % Configuration settings
   properties ( SetAccess = private, GetAccess = public )
      fft_size;            % Size
      fft_size_log2;       % log2(Size)
      num_stages;          % Number of r4/r2 stages
      
      is_ifft     = [];    % Are we doing ifft (conjugate twiddle factors)
      is_dit      = [];    % Are we doing Decimation in Time fft
      
      is_rev_inp = [];     % Reordering on the  input samples
      is_rev_out = [];     % Reordering on the output samples
      
      reorder    = [];     % Reordering is done inside the FFT processor 
                           % either when the input samples are read 
                           % or the output samples re written into ram   
   end % properties   
   
   % Processing variables
   properties ( SetAccess = private, GetAccess = private )
      buf = [];         % Buffer which represents banks
      w   = [];         % Array of the twiddle factors   
      radix_dir = [];   % fft radix for each processing stage (4 or 2)
   end % properties    
   
   methods
      % Class constructor
      function this = fft_c( prm )
         assert( isscalar( prm ), [this.class_id, '(): Incorrect parameter specification.']);
         
         % Save constructor parameters
         this.run_prm = prm;

         % Save the name of the folder which we are running from
         [this.run_fpath, ~, ~] = fileparts( mfilename('fullpath') );         
         
         % radix for each stage
         this.fft_size      = this.run_prm.size;
         this.fft_size_log2 = log2( this.fft_size );
         
         % only 2^N fft sizes are supported
         assert( this.fft_size == 2^this.fft_size_log2 );
         
         switch( this.dm_ways )
            case 2
               this.radix_dir =  2 .* ones( floor( this.fft_size_log2 ), 1 );
               
            case 4
               if(     strcmp( this.run_prm.stages, 'r2r4' ))
                  % Residual radix-2 stage comes first
                  this.radix_dir = [4 .* ones( floor( this.fft_size_log2 / 2 ), 1 ); ...
                                    2 .* ones(   mod( this.fft_size_log2,  2 ), 1 )];   
               elseif( strcmp( this.run_prm.stages, 'r4r2' ))
                  % Residual radix-2 stage comes last
                  this.radix_dir = [2 .* ones(   mod( this.fft_size_log2,  2 ), 1 ); ...
                                    4 .* ones( floor( this.fft_size_log2 / 2 ), 1 )];   
               else
                  error( [this.class_id, '(): Incorrect configuration of the stages.']);
               end
                  
            otherwise
               error( [this.class_id, '(): Incorrect dm configuration.']);
         end % switch( this.dm_ways )
         
         this.num_stages = numel( this.radix_dir );
         
         % ifft?
         this.is_ifft = this.run_prm.ifft;
         
         % Configuration
         switch( this.run_prm.conf )
            case 'dif'
               this.is_dit = false;
               
            case 'dit'
               this.is_dit = true;
               
            otherwise
               error( [this.class_id, '(): Incorrect fft configuration.'] );
         end % switch(         
         
         % Allocate buffer (tbd)
         this.buf = zeros( this.fft_size / this.dm_ways, ...
                           this.dm_ways,                 ...
                           this.num_stages + 1 );        ...
         
         % Initialize LUT with twiddle factors (tbd)
         this.w = exp(( 2. * pi * 1j / this.fft_size ) .* ( 0 : this.fft_size - 1 ).' );
         
         % Address generators
         switch( this.run_prm.order )
            case 'rn'   % first - digit-reverse; last - natural
               this.is_rev_inp = true;
               this.is_rev_out = false;

            case 'nr'   % first - natural; last - digit-reverse
               this.is_rev_inp = false;
               this.is_rev_out = true;
               
            otherwise
               error( [this.class_id, '(): Incorrect fft i/o order.'] );
         end % switch(  
         
         this.reorder = this.run_prm.reorder; 
         
      end % function this = fft_c( ...

   end % methods
   
   methods ( Access = private )
      % Decompose index value into the digits which correspond to the vector of radixes 
      function dig = radix_dig( this, val, rad ) 
         assert( 0 <= val && val < this.fft_size, [this.class_id, '.radix_dig(): Incorrect input value.'] );
         
         bin = de2bi( val, this.fft_size_log2, 2, 'right-msb' );
         
         dig = zeros( size( rad ));
         bin_idx_str = 1;
         
         for rad_idx = 1:numel( rad )
            bin_idx_end = bin_idx_str + log2( rad( rad_idx )) - 1;
            
            dig( rad_idx ) = bi2de( bin( bin_idx_str:bin_idx_end ), 2, 'right-msb' );
            
            bin_idx_str = bin_idx_end + 1;
         end
      end % function dig = radix_dig(
      
      % Compose value from the digits which correspond to the vector of radixes 
      function val = radix_val( this, dig, rad ) 
         bin = zeros( 1, this.fft_size_log2 );
         
         bin_idx_str = 1;
         for rad_idx = 1:numel( rad )
            bin_idx_end = bin_idx_str + log2( rad( rad_idx )) - 1;
            
            bin( bin_idx_str:bin_idx_end ) = de2bi( dig( rad_idx ), log2( rad( rad_idx )), 2, 'right-msb' );
            
            bin_idx_str = bin_idx_end + 1;
         end 
         
         val = bi2de( bin, 2, 'right-msb' );
      end % function val = radix_val(
      
      % Butterfly configuration
      function out = btf( this, stg )
         assert( stg <= this.num_stages );
         
         out = repmat( struct( ...
            'bundle',[],   ...
            'radix', [],   ...
            'i_idx', [],   ...
            'o_idx', [],   ...
            'w_idx', [],   ...
            'i_adr', [],   ...
            'i_way', [],   ...
            'o_adr', [],   ...
            'o_way', [],   ...
            'i_way_flat', [],   ...
            'o_way_flat', [],   ...            
            'dummy', [] ), this.fft_size / max( this.radix_dir ), 1 );
         
         if( this.is_rev_inp )
            % radix array to construct butterfly index
            radix_prm = flip( this.radix_dir );       
            
            % position in the radix array which corresponds to the butterfly inputs
            btfly_pos = stg;                          
         elseif( this.is_rev_out )
            % radix array to construct butterfly index
            radix_prm = this.radix_dir;
            
            % position in the radix array which corresponds to the butterfly inputs
            btfly_pos = this.num_stages - stg + 1;
         end  
         
         % Radix of the butterfly
         btfly_rad = radix_prm( btfly_pos );
         
         % from the radix array puncture out the position which corresponds to the butterfly inputs
         if(     btfly_pos == 1 )
            radix_btf_idx = radix_prm( 2:this.num_stages );
         elseif( btfly_pos == this.num_stages )
            radix_btf_idx = radix_prm( 1:this.num_stages - 1 );
         else
            radix_btf_idx = radix_prm( [( 1 : btfly_pos - 1 ), ( btfly_pos + 1 : this.num_stages )]);
         end   
         
         % way indexes for the butterfly bundle
         way_idx = 0:max( this.radix_dir ) - 1;
         out( 1 ).i_way_flat = zeros( this.fft_size, 1 );
         out( 1 ).o_way_flat = zeros( this.fft_size, 1 );
         
         for btf_idx = 1:numel( out )
            out( btf_idx ).radix = btfly_rad;
            
            % bundling of r2 butterflies when they appear in r4 + 2r2 configuration
            if( btfly_rad == 2 && btfly_rad ~= max( this.radix_dir ))
               out( btf_idx ).bundle = 2; 
            else
               out( btf_idx ).bundle = 1; 
            end
            
            out( btf_idx ).i_idx = zeros( btfly_rad, out( btf_idx ).bundle );
            out( btf_idx ).o_idx = zeros( btfly_rad, out( btf_idx ).bundle );
            
            for btf_idx_in_pair = 1:out( btf_idx ).bundle
               if( out( btf_idx ).bundle == 1 )
                  btf_idx_full = ( btf_idx - 1 );
               else
                  %btf_idx_full = ( btf_idx - 1 ) + numel( out ) * ( btf_idx_in_pair - 1 );
                  
                  btf_idx_full = ( btf_idx - 1 ) * out( btf_idx ).bundle + ( btf_idx_in_pair - 1 );
               end
               
               % Decompose butterfly index
               btf_idx_dig = this.radix_dig( btf_idx_full, radix_btf_idx ); 
               
               % butterfly ports
               for btf_io_idx = 1:btfly_rad
                  btf_io_idx_full = out( btf_idx ).bundle * ( btf_idx_in_pair - 1 ) + btf_io_idx;
                  
                  % combine butterfly index and port index to get io address 
                  if(     btfly_pos == 1 )
                     comb_dig = [btf_io_idx - 1; btf_idx_dig];
                  elseif( btfly_pos == this.num_stages )
                     comb_dig = [btf_idx_dig; btf_io_idx - 1];
                  else
                     comb_dig = [btf_idx_dig( 1 : btfly_pos - 1 ); ...
                                 btf_io_idx - 1;                  ...
                                 btf_idx_dig( btfly_pos : this.num_stages - 1 )];
                  end                  
                  
                  idx_temp = this.radix_val( comb_dig, radix_prm );
                  
                  % Perform input or output digit-reverse reordering if required
                  if( this.is_rev_inp && this.reorder && ( stg == 1               ) || ...
                      this.is_rev_out && this.reorder && ( stg == this.num_stages ))
                     idx_reorder = this.radix_val( ...
                           flip( this.radix_dig( idx_temp, this.radix_dir )), ...
                           flip( this.radix_dir ));

                     if( this.is_inp_rev )   
                        i_idx = idx_reorder;
                        o_idx = idx_temp;
                     else
                        i_idx = idx_temp;
                        o_idx = idx_reorder;
                     end
                  else
                     i_idx = idx_temp;
                     o_idx = idx_temp;
                  end       

                  out( btf_idx ).i_idx( btf_io_idx, btf_idx_in_pair ) = i_idx;
                  out( btf_idx ).o_idx( btf_io_idx, btf_idx_in_pair ) = o_idx;
                  
                  way_i_idx = circshift( way_idx, -mod( floor( i_idx / max( this.radix_dir ) ), max( this.radix_dir )));
                  way_o_idx = circshift( way_idx, -mod( floor( o_idx / max( this.radix_dir ) ), max( this.radix_dir )));
                  
                  out( 1 ).i_way_flat( i_idx + 1 ) = way_i_idx( mod( i_idx, max( this.radix_dir )) + 1 );
                  out( 1 ).o_way_flat( o_idx + 1 ) = way_o_idx( mod( o_idx, max( this.radix_dir )) + 1 );                  
                  
               end % for btf_io_idx = ...
            end % for btf_idx_in_pair = 
            
            % Update way indexes for the next butterfly bundle
            %way_idx = circshift( way_idx, 1 );
            
         end % for btf_idx =
         
      end % function out = btf(
   end % methods
   
   methods ( Access = public )
      % Plot fft processing graph
      function none = graph( this, ax ) 
         hold( ax, 'on' );
         set( ax, ...
            'NextPlot', 'add', ...
            'Ydir',     'reverse', ...
            'XLim',     [0, this.num_stages], ...
            'YLim',     [0, this.fft_size - 1] ); 
         
         ax.XAxis.Visible = 'off';
         ax.YAxis.Visible = 'off';
         
         % Color map for the interconnection between the RAM tiles and processing elements
         way_col_map = jet( max( this.radix_dir ));
         
         % Cycle through the processing stages
         for stg_idx = 1:this.num_stages
            btf_vec = this.btf( stg_idx ); % Create vector of the processing elements for the current stage
            
            % Y-positions of the markers which represent buttrflies
            btf_pos = linspace( 0, this.fft_size - 1,  numel( btf_vec ) * btf_vec( 1 ).bundle + 2 );
            btf_pos = btf_pos( 2:end-1 );            
            
            for btf_idx = 1:numel( btf_vec )
               for btf_idx_in_pair = 1:btf_vec( btf_idx ).bundle
                  btf_idx_full = ( btf_idx - 1 ) * btf_vec( btf_idx ).bundle + btf_idx_in_pair;
                  
                  % Marker for the butterfly
                  plot( ax, stg_idx - 0.5, btf_pos( btf_idx_full ), ...
                     'LineStyle',         'none',   ...
                     'Marker',            'o',      ...
                     'MarkerEdgeColor',   'r',      ...
                     'MarkerFaceColor',   'none',   ...  
                     'MarkerSize',  20 );   
                  
                  % Bundle butterfly markers 
                  if( btf_vec( btf_idx ).bundle && btf_idx_in_pair == btf_vec( btf_idx ).bundle )
                     btf_idx_str = ( btf_idx - 1 ) * btf_vec( btf_idx ).bundle + 1;
                     btf_idx_end = ( btf_idx - 1 ) * btf_vec( btf_idx ).bundle + btf_vec( btf_idx ).bundle;
                     plot( ax, [stg_idx - 0.5, stg_idx - 0.5], [btf_pos( btf_idx_str ), btf_pos( btf_idx_end )], 'r-' );
                  end                  
                  
                  for io_idx = 1:btf_vec( btf_idx ).radix
                     i_idx = btf_vec( btf_idx ).i_idx( io_idx, btf_idx_in_pair );
                     i_way = btf_vec( btf_idx ).i_way( io_idx, btf_idx_in_pair );
                     o_idx = btf_vec( btf_idx ).o_idx( io_idx, btf_idx_in_pair );
                     o_way = btf_vec( btf_idx ).o_way( io_idx, btf_idx_in_pair );
                     
                     % Marker for the input RAM cell
                     plot( ax, stg_idx - 1, i_idx, ...
                        'LineStyle',         'none',   ...
                        'Marker',            'square', ...
                        'MarkerEdgeColor',   way_col_map( i_way, : ), ...
                        'MarkerFaceColor',   'none',   ...  
                        'MarkerSize',  18 );
                     
                     % Marker for the output RAM cell
                     plot( ax, stg_idx - 0, o_idx, ...
                        'LineStyle',         'none',   ...
                        'Marker',            'square', ...
                        'MarkerEdgeColor',   way_col_map( o_way, : ), ...
                        'MarkerFaceColor',   'none',   ...  
                        'MarkerSize',  20 );  
                     
                     % Connection line from the input RAM cell into the processing butterfly
                     plot( ax, ...
                        [stg_idx - 1.0, stg_idx - 0.5], ...
                        [i_idx, btf_pos( btf_idx_full )], ... 
                        'LineStyle',  '-',   ...
                        'Color',      way_col_map( i_way, : ));
                     
                     % Connection line from the processing butterfly into the output RAM cell
                     plot( ax, ...
                        [stg_idx - 0.5, stg_idx      ], ...
                        [btf_pos( btf_idx_full ), o_idx], ...
                        'LineStyle',  '-',   ...
                        'Color',      way_col_map( i_way, : ));    
                     
                  end % for io_idx = 1:btf_this( btf_idx ).radix
               end % for btf_idx_in_pair = 1:bf_this( btf_idx ).bundle
            end % for btf_idx = 1:numel( bf_this )
         end % for stg_idx = 1:this.num_stages
         
         none = [];
      end % function none = graph( this )
   end % methods
end % classdef fft_c
