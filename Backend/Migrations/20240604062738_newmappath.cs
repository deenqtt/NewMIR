using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace Backend.Migrations
{
    /// <inheritdoc />
    public partial class newmappath : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "PgmFile",
                table: "Maps");

            migrationBuilder.DropColumn(
                name: "YamlFile",
                table: "Maps");

            migrationBuilder.AddColumn<string>(
                name: "PgmFilePath",
                table: "Maps",
                type: "TEXT",
                nullable: false,
                defaultValue: "");

            migrationBuilder.AddColumn<string>(
                name: "YamlFilePath",
                table: "Maps",
                type: "TEXT",
                nullable: false,
                defaultValue: "");
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "PgmFilePath",
                table: "Maps");

            migrationBuilder.DropColumn(
                name: "YamlFilePath",
                table: "Maps");

            migrationBuilder.AddColumn<byte[]>(
                name: "PgmFile",
                table: "Maps",
                type: "BLOB",
                nullable: false,
                defaultValue: new byte[0]);

            migrationBuilder.AddColumn<byte[]>(
                name: "YamlFile",
                table: "Maps",
                type: "BLOB",
                nullable: false,
                defaultValue: new byte[0]);
        }
    }
}
